#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

const char* ssid = "NLBVN";
const char* password = "66666666";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;
WebsocketsServer socket_server;

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;

// ================= CẤU HÌNH SERVO SG90 =================
// Dây cam/vàng (tín hiệu) nối chân dưới. Dây đỏ nối 5V NGUỒN RIÊNG, dây nâu/đen nối GND chung.
#define SERVO_PIN        13      // chân tín hiệu servo (trước đây là chân relay). Nếu nạp code bị lỗi, đổi sang 13 hoặc 14
#define SERVO_CH         2      // kênh LEDC của servo (kênh 0 đã dùng cho xung clock camera, đừng đặt 0)
#define SERVO_FREQ       50     // SG90 dùng xung 50 Hz
#define SERVO_RES        16     // độ phân giải PWM 16 bit
#define SERVO_MIN_US     500    // độ rộng xung ứng với 0 độ (µs)
#define SERVO_MAX_US     2400   // độ rộng xung ứng với 180 độ (µs)
#define SERVO_LOCK_ANGLE 0      // góc khi cửa KHÓA
#define SERVO_OPEN_ANGLE 90     // góc khi cửa MỞ (chỉnh theo cơ cấu khóa của bạn)
#define SERVO_SETTLE_MS  700    // thời gian chờ servo quay xong
#define SERVO_HOLD       false  // false: nhả xung sau khi quay xong (hết rung, đỡ nóng); true: giữ xung để servo giữ lực

// ================= KẾT NỐI WEB PYTHON =================
#define ALLOW_REMOTE_OPEN true             // false: tắt hẳn lệnh mở cửa từ xa
#define REMOTE_KEY "12"      // BẮT BUỘC đổi; phải trùng REMOTE_KEY trong app.py của web Python
volatile bool remote_open_request = false; // web đặt cờ này, loop() sẽ mở cửa
char last_name[32] = "";                   // tên người nhận diện gần nhất
bool last_name_valid = false;
unsigned long last_recognised_millis = 0;

unsigned long door_opened_millis = 0;
long interval = 5000;           // giữ cửa mở trong ... mili giây
bool face_recognised = false;
bool door_is_open = false;
bool servo_active = false;
unsigned long servo_release_millis = 0;

void app_facenet_main();
void app_httpserver_init();

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;


static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

// ================= ĐIỀU KHIỂN SERVO =================
// Quay servo tới góc angle (0-180). Dùng PWM LEDC trực tiếp, không cần thư viện ngoài
// nên không xung đột với timer của camera.
static void servo_write_angle(int angle)
{
  angle = constrain(angle, 0, 180);
  uint32_t us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = (uint32_t)(((uint64_t)us * ((1UL << SERVO_RES) - 1)) / 20000UL);  // chu kỳ 20 ms
  ledcWrite(SERVO_CH, duty);
  servo_active = true;
  servo_release_millis = millis() + SERVO_SETTLE_MS;
}

static void close_door()
{
  servo_write_angle(SERVO_LOCK_ANGLE);
  door_is_open = false;
  Serial.println("Door Locked");
}

// Mở cửa (dùng chung cho nhận diện mặt và lệnh từ xa). Trả về true nếu cửa vừa được mở mới.
static bool open_door_core()
{
  bool was_closed = !door_is_open;
  if (was_closed) {
    servo_write_angle(SERVO_OPEN_ANGLE);
    door_is_open = true;
    Serial.println("Door Unlocked");
  }
  door_opened_millis = millis();           // gia hạn thời gian giữ cửa mở
  return was_closed;
}

// Gọi liên tục trong loop(): tự khóa cửa khi hết thời gian và nhả xung servo sau khi quay xong.
// Không dùng delay() nên không làm đứng luồng video.
static void door_service()
{
  unsigned long now = millis();
  if (remote_open_request) {               // lệnh mở cửa từ web Python
    remote_open_request = false;
    open_door_core();
    Serial.println("Remote open request");
  }
  if (door_is_open && (now - door_opened_millis) > (unsigned long)interval) {
    close_door();
  }
  if (servo_active && !SERVO_HOLD && (long)(now - servo_release_millis) >= 0) {
    ledcWrite(SERVO_CH, 0);   // ngừng phát xung: servo hết rung, đỡ tốn dòng
    servo_active = false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Servo: khởi tạo PWM và đưa về vị trí KHÓA khi cấp điện
  ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
  servo_write_angle(SERVO_LOCK_ANGLE);
  delay(SERVO_SETTLE_MS);
  door_service();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

// ----- /status: trạng thái cho web Python (JSON) -----
static esp_err_t status_handler(httpd_req_t *req) {
  char name[32];
  strncpy(name, last_name, sizeof(name));
  name[sizeof(name) - 1] = 0;
  for (char *c = name; *c; c++) {          // tránh ký tự làm hỏng JSON
    if (*c == '"' || *c == '\\' || *c < 32) *c = '_';
  }
  long ago = last_name_valid ? (long)((millis() - last_recognised_millis) / 1000) : -1;
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"door\":\"%s\",\"faces\":%d,\"last_name\":\"%s\",\"last_seen_s_ago\":%ld,\"uptime_s\":%lu,\"rssi\":%d}",
           door_is_open ? "open" : "closed", st_face_list.count, name, ago, millis() / 1000UL, WiFi.RSSI());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

// ----- /open?key=...: mở cửa từ xa, bắt buộc đúng REMOTE_KEY -----
static esp_err_t open_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  char query[96] = {0}, key[64] = {0};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "key", key, sizeof(key));
  }
  if (!ALLOW_REMOTE_OPEN || strcmp(key, REMOTE_KEY) != 0) {
    httpd_resp_set_status(req, "403 Forbidden");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"forbidden\"}", HTTPD_RESP_USE_STRLEN);
  }
  remote_open_request = true;
  return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

httpd_uri_t status_uri = {
  .uri       = "/status",
  .method    = HTTP_GET,
  .handler   = status_handler,
  .user_ctx  = NULL
};

httpd_uri_t open_uri = {
  .uri       = "/open",
  .method    = HTTP_GET,
  .handler   = open_handler,
  .user_ctx  = NULL
};

void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &open_uri);
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
  Serial.printf("Da doc %d khuon mat tu flash\n", st_face_list.count);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    client.send(add_face); //send face to browser
    head = head->next;
  }
  return ESP_OK;
}

static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
  return ESP_OK;
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);
  }
}

void open_door(WebsocketsClient &client) {
  if (open_door_core()) {
    client.send("door_open");
  }
}

void loop() {
  auto client = socket_server.accept();
  door_service();                          // vẫn tự khóa cửa khi không có trình duyệt nào kết nối

  if (!client.available()) {
    return;                                // chưa có client: không cấp phát bộ nhớ ảnh nữa
  }

  client.onMessage(handle_message);
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.image = image_matrix->item;

  send_face_list(client);
  client.send("STREAMING");

  while (client.available()) {
    client.poll();

    door_service();

    fb = esp_camera_fb_get();

    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
    {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

      if (out_res.net_boxes)
      {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
        {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (g_state == START_DETECT) {
            client.send("FACE DETECTED");
          }

          if (g_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);
              send_face_list(client);

            }
          }

          if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f)
            {
              char recognised_message[64];
              strncpy(last_name, f->id_name, sizeof(last_name) - 1);
              last_name[sizeof(last_name) - 1] = 0;
              last_name_valid = true;
              last_recognised_millis = millis();
              sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
              open_door(client);
              client.send(recognised_message);
            }
            else
            {
              client.send("FACE NOT RECOGNISED");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }

      }
      else
      {
        if (g_state != START_DETECT) {
          client.send("NO FACE DETECTED");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
        client.send("DETECTING");
      }

    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }

  dl_matrix3du_free(image_matrix);         // giải phóng bộ nhớ ảnh khi client ngắt kết nối
}

#include "BT_task.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "Stepper.h"
#include "Servo.h"
#include "I2C_screen.h"
#include "rotaryEncoder.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#define WIFI_SSID "ESP32_AP"
#define WIFI_PASS "12345678"
#define WIFI_CHANNEL 1
#define MAX_CONN 4

#define SERVER_IP "192.168.4.2" // IP fixe de notre Android sur le SoftAP
#define SERVER_PORT 5005

// MOTOR 1
#define M1_STEP GPIO_NUM_19
#define M1_DIR GPIO_NUM_18
// MOTOR 2
#define M2_STEP GPIO_NUM_5
#define M2_DIR GPIO_NUM_17
// MOTOR 3
#define M3_STEP GPIO_NUM_16
#define M3_DIR GPIO_NUM_4
// MOTOR 4
#define M4_STEP GPIO_NUM_2
#define M4_DIR GPIO_NUM_15

#define EN_MOTOR GPIO_NUM_23

// HOME
#define HOME1 GPIO_NUM_34
#define HOME2 GPIO_NUM_35
#define HOME3 GPIO_NUM_32
#define HOME4 GPIO_NUM_33

// SERVO
#define SERVOA GPIO_NUM_13
#define SERVOB GPIO_NUM_14
#define SA_OPEN 55
#define SA_CLOSE -70

#define SB_OPEN 55
#define SB_CLOSE -70

#define MOTOR_MS 10
#define SERVO_MS 500

// IHM
extern uint8_t switch_state;
extern uint16_t rotary_pos;
extern uint8_t rotFlag_A;
extern uint8_t rotFlag_B;
uint8_t last_rotary_pos;
SSD1306_t screen;
uint8_t pageIHM = 0;

const UBaseType_t taskPriority = 1;
#define ACCEL 200 // 200
#define SPEED 150 // 150

Motor M1 = {HOME1, M1_STEP, M1_DIR, RPM_TO_RADpS(ACCEL), RPM_TO_RADpS(SPEED)};
Motor M2 = {HOME2, M2_STEP, M2_DIR, RPM_TO_RADpS(ACCEL), RPM_TO_RADpS(SPEED)};
Motor M3 = {HOME3, M3_STEP, M3_DIR, RPM_TO_RADpS(ACCEL), RPM_TO_RADpS(SPEED)};
Motor M4 = {HOME4, M4_STEP, M4_DIR, RPM_TO_RADpS(ACCEL), RPM_TO_RADpS(SPEED)};

static const char *TAG_WIFI = "SoftAP";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  if (event_id == WIFI_EVENT_AP_STACONNECTED)
  {
    wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG_WIFI, "Station connectée: " MACSTR ", AID=%d", MAC2STR(event->mac), event->aid);
  }
  else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
  {
    wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG_WIFI, "Station déconnectée: " MACSTR ", AID=%d", MAC2STR(event->mac), event->aid);
  }
}

void wifi_init_softap(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));

  wifi_config_t wifi_config = {
      .ap = {
          .ssid = WIFI_SSID,
          .ssid_len = strlen(WIFI_SSID),
          .channel = WIFI_CHANNEL,
          .password = WIFI_PASS,
          .max_connection = MAX_CONN,
          .authmode = WIFI_AUTH_WPA2_PSK},
  };

  if (strlen(WIFI_PASS) == 0)
  {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG_WIFI, "SoftAP lancé: SSID:%s, mot de passe:%s", WIFI_SSID, WIFI_PASS);
}

bool Enable = true;
void homeAllSteppers()
{
  gpio_set_level(EN_MOTOR, 0);
  moveServo(SERVOB, SB_OPEN);
  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  home(&M1);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  home(&M2);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  home(&M3);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  home(&M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  moveServo(SERVOA, SA_CLOSE);
}

// place la face physique R en U (la face B se retrouve en F)
void orientR()
{

  // place la face physique R en U
  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(180, CW, &M1, &M3);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  // place la face B en F
  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CCW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
}

// place la face physique F en U (la face R se retrouve en F)
void orientF()
{

  // place la face physique F en U
  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(180, CW, &M1, &M3);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CCW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  // place la face R en F
  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CCW, &M1, &M3);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CW, &M1, &M3);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
}

void up(int degree, int sens)
{
  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CCW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CCW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  turn(degree, sens, &M1);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  if (degree == 90)
  {
    turn(90, !sens, &M1);
    vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  }

  turnMultiple(90, CW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
}

void down(int degree, int sens)
{
  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CCW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CCW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  turn(degree, sens, &M3);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  if (degree == 90)
  {
    turn(degree, !sens, &M3);
    vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  }

  turnMultiple(90, CW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOA, SA_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);

  moveServo(SERVOB, SB_OPEN);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
  turnMultiple(90, CW, &M2, &M4);
  vTaskDelay(MOTOR_MS / portTICK_PERIOD_MS);
  moveServo(SERVOB, SB_CLOSE);
  vTaskDelay(SERVO_MS / portTICK_PERIOD_MS);
}

void StringToMoves(uint8_t *moves)
{
  printf("TRAME : %s\n", moves);
  int prev_sens = CW;
  int sens = CW;
  int prev_degree = 90;
  int degree = 90;
  int prev_opti = 0;

  char frame[50];
  int j = 0;
  int inFrame = 0;

  for (int i = 0; moves[i] != '\0'; i++)
  {
    if (moves[i] == '>')
    {
      inFrame = 1;
      j = 0;
    }
    else if (moves[i] == '<' && inFrame)
    {
      frame[j] = '\0'; // fin de frame
      inFrame = 0;

      // ex "LEFT:1"
      printf("FRAME: %s\n", frame);

      // Separer direction et valeur
      char *direction = strtok(frame, ":");
      char *value = strtok(NULL, ":");
      
      
      if (strcmp(direction, "RELEASE") != 0)
      {
        
        if (!direction || !value)
        {
          printf("Invalid frame format!\n");
          continue;
        }
      }

      // Exemple: '1' => CW et 90deg, '2' => 180°, 'm' => CCW et 90deg
      sens = CW;
      degree = 90;
      if (strcmp(value, "2") == 0)
        degree = 180;
      if (strcmp(value, "m") == 0)
        sens = CCW;

      if (strcmp(direction, "UP") == 0)
      {
        printf("UP %s %d°\n", sens == CCW ? "CCW" : "CW", degree);
        up(degree, sens);
        break;
      }
      else if (strcmp(direction, "DOWN") == 0)
      {
        printf("DOWN %s %d°\n", sens == CCW ? "CCW" : "CW", degree);
        down(degree, sens);
        break;
      }
      else if (strcmp(direction, "LEFT") == 0)
      {
        printf("LEFT %s %d°\n", sens == CCW ? "CCW" : "CW", degree);
        orientR();
        down(degree, sens);
        break;
      }
      else if (strcmp(direction, "RIGHT") == 0)
      {
        printf("RIGHT %s %d°\n", sens == CCW ? "CCW" : "CW", degree);
        orientR();
        up(degree, sens);
        break;
      }
      else if (strcmp(direction, "FRONT") == 0)
      {
        printf("FRONT %s %d°\n", sens == CCW ? "CCW" : "CW", degree);
        orientF();
        up(degree, sens);
        break;
      }
      else if (strcmp(direction, "BACK") == 0)
      {
        printf("BACK %s %d°\n", sens == CCW ? "CCW" : "CW", degree);
        orientF();
        down(degree, sens);
        break;
      }
      else if (strcmp(direction, "SHOW") == 0)
      {
        char arg_char = value[0];
        if (arg_char == 1)
        { // Présentation face F
          printf("SHOW : %d°\n", arg_char);
        }
        else if (arg_char == 2)
        { // Présentation face B
          printf("SHOW : %d°\n", arg_char);
        }
        else if (arg_char == 3)
        { // Présentation face L
          printf("SHOW : %d°\n", arg_char);
        }
        else if (arg_char == 4)
        { // Présentation face R
          printf("SHOW : %d°\n", arg_char);
        }
        else if (arg_char == 5)
        { // Présentation face D
          printf("SHOW : %d°\n", arg_char);
        }
        else if (arg_char == 6)
        { // Présentation face U
          printf("SHOW : %d°\n", arg_char);
        }
        else if (arg_char == 7)
        { // Présentation face R
          printf("SHOW : %d°\n", arg_char);
        }
        break;
      }
      else if (strcmp(direction, "RELEASE") == 0)
      {
        printf("RELEASE\n");
      
        break;
      }
      else
      {
        printf("Unknown direction: %s\n", direction);
      }

      prev_sens = sens;
      prev_degree = degree;
    }
    else if (inFrame)
    {
      frame[j++] = moves[i];
    }
  }
}
/*void taskBT(void *pvParameters){
  // static int cnt = 0;
  while (1){
    xSemaphoreTake(dataAvailable, portMAX_DELAY);
    uint8_t *moves = getBTbuffer();
    StringToMoves(moves);
  }
}*/
void taskIHM(void *pvParameters)
{
  initScreen(&screen);
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  ssd1306_clear_screen(&screen, 0);
  updatePageIHM(&screen, pageIHM);
  while (1)
  {
    xSemaphoreTake(updateIHM, portMAX_DELAY);
    printf("Update IHM: \n");
    if (switch_state)
    {
      printf("BP Pressed: \n");

      switch (pageIHM)
      {
      case 0: // OPEN
        moveServo(SERVOB, SB_OPEN);
        moveServo(SERVOA, SA_OPEN);
        break;
      case 1: // CLOSE
        moveServo(SERVOB, SB_CLOSE);
        moveServo(SERVOA, SA_CLOSE);
        break;
      case 2: // HOME
        homeAllSteppers();
        break;
      case 3: // MIX

        break;
      }
    }
    else
    {
      printf("Rotary State: %d\n", rotary_pos);
      pageIHM++;
      if (pageIHM == 4)
        pageIHM = 0;
      updatePageIHM(&screen, pageIHM);
    }
    switch_state = 0;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

#define TCP_BUFFER_SIZE 128 // Re-define if not in the original headers

void tcp_client_task(void *pvParameters)
{
  // Use a fixed-size buffer on the stack for simplicity and safety
  char rx_buffer[TCP_BUFFER_SIZE];
  int sock;
  struct sockaddr_in dest_addr;

  dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(SERVER_PORT);

  while (1)
  {
    // --- 1. Establish Connection ---
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
      ESP_LOGE(TAG_WIFI, "Erreur socket: %d", errno);
      goto reconnect;
    }

    ESP_LOGI(TAG_WIFI, "Connexion au serveur TCP...");
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
    {
      ESP_LOGE(TAG_WIFI, "Erreur connexion: %d", errno);
      close(sock);
      goto reconnect;
    }

    ESP_LOGI(TAG_WIFI, "Connecté au serveur ! Prêt à recevoir.");

    // --- 2. Inner Loop: Listen, Execute, and Acknowledge ---
    while (1)
    {
      memset(rx_buffer, 0, TCP_BUFFER_SIZE); // Clear the buffer

      // Wait to receive a command (e.g., "U" or "U'")
      int len = recv(sock, rx_buffer, TCP_BUFFER_SIZE - 1, 0);

      if (len <= 0)
      {
        // len < 0 is an error, len == 0 is connection closed
        if (len < 0)
          ESP_LOGE(TAG_WIFI, "Erreur lors de la réception: %d", errno);
        else
          ESP_LOGW(TAG_WIFI, "Serveur Android déconnecté.");
        break; // Exit inner loop to trigger reconnection
      }

      // Command received
      rx_buffer[len] = 0; // Null-terminate
      ESP_LOGI(TAG_WIFI, "Reçu: %s", rx_buffer);

      // Execute the move (The 'U' logic is inside this function)
      StringToMoves((uint8_t *)rx_buffer);

      // --- 3. Send Acknowledgment (ACK) ---

      // The ACK is the received command string itself.
      // We append '\n' for proper line-based reading on the Android side.
      size_t ack_len = strlen(rx_buffer);

      // Check if there is space for a newline and null-terminator
      if (ack_len < TCP_BUFFER_SIZE - 2)
      {
        rx_buffer[ack_len] = '\n';
        ack_len++;
        rx_buffer[ack_len] = 0; // Re-terminate the string
      }
      else
      {
        // Handle buffer too small if necessary
        ESP_LOGW(TAG_WIFI, "Buffer size too small for ACK newline.");
      }

      int err = send(sock, rx_buffer, ack_len, 0);
      if (err < 0)
      {
        ESP_LOGE(TAG_WIFI, "Erreur lors de l'envoi de l'ACK: %d", errno);
        break; // Exit inner loop on send error
      }
      ESP_LOGI(TAG_WIFI, "ACK envoyé: %s", rx_buffer);

      // Loop continues, waiting for the next command
    }

    // --- 4. Clean up and Reconnect ---
    if (sock != -1)
    {
      close(sock);
    }

  reconnect:
    ESP_LOGW(TAG_WIFI, "Déconnexion. Tentative de reconnexion dans 2 secondes...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  wifi_init_softap();
  xTaskCreate(tcp_client_task, "tcp_client", 8192, NULL, 5, NULL);
  // initBT("Rubik's Solveur");
  initRotaryEncoder();
  printf("init done\n");
  /*xTaskCreate(
    taskBT,       // Entry function of the task
    "taskBT",     // Name of the task
    1024,        // The number of words to allocate for use as the task's stack (arbitrary size enough for this task)
    NULL,         // No parameter passed to the task
    taskPriority, // Priority of the task
    NULL);        // No handle*/
  xTaskCreate(
      taskIHM,      // Entry function of the task
      "taskIHM",    // Name of the task
      10000,        // The number of words to allocate for use as the task's stack (arbitrary size enough for this task)
      NULL,         // No parameter passed to the task
      taskPriority, // Priority of the task
      NULL);
  printf("task done\n");
  gpio_reset_pin(EN_MOTOR);
  gpio_set_direction(EN_MOTOR, GPIO_MODE_OUTPUT);
  initStepper(&M1, &M2, &M3, &M4);
  initServo(SERVOA, SERVOB);
  while (1)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (rotary_pos != last_rotary_pos)
    {
      printf("rotary pos : %d\n", rotary_pos);
      last_rotary_pos = rotary_pos;
    }
  }
}

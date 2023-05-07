#include <Wire.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <Arduino.h>
#include <soc/soc.h>           // Disable brownout problems
#include <soc/rtc_cntl_reg.h>  // Disable brownout problems
#include <driver/rtc_io.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>
#include <LiquidCrystal_I2C.h>
//Provide the token generation process info.
#include <addons/TokenHelper.h>

//LCD I2C pin config
#define I2C_SDA 14
#define I2C_SCL 15
TwoWire I2CLCD = TwoWire(0);
LiquidCrystal_I2C lcd(0x27, 16, 2);

//e18 Distance sensor initialize
//#define distanceSensor 12

//relay module initialize
#define relay 13

//Replace with your network credentials
const char* ssid = "CEMARA-AP";
const char* password = "RTYUIOP999";

// //Replace with your network credentials
// const char* ssid = "FMC IndiHome ZTE - A";
// const char* password = "wireless";

// Insert Firebase project API Key
#define API_KEY "AIzaSyDtLZOtHS-s2iyWZmjnxB4WMHqEOcHCayo"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "smartdoor@gmail.com"
#define USER_PASSWORD "smartdoor"

// Insert Firebase storage bucket ID e.g bucket-name.appspot.com
#define STORAGE_BUCKET_ID "smart-door-esp32-cam.appspot.com"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "smart-door-esp32-cam-default-rtdb.asia-southeast1.firebasedatabase.app/" 

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/data/photo.jpg"

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

bool takeNewPhoto = true;
bool takePhotoAgain = true;
//Define Firebase Data objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig configF;
int intAppButton = 0;
int intCamera = 0;
String stringDesc;
bool program = false;
bool signupOK = false;
bool taskCompleted = false;

void initLCD(void){
  Wire.begin(I2C_SDA, I2C_SCL);
  I2CLCD.begin(I2C_SDA, I2C_SCL, 100000);
  lcd.init();              
  lcd.backlight();
}

// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSaveSpiffs(void){
  camera_fb_t * fb = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly
  do{
    // Take a photo with the camera
    Serial.println("Taking a photo...");
    pinMode(GPIO_NUM_4, OUTPUT);
    digitalWrite(GPIO_NUM_4, HIGH);
    rtc_gpio_hold_dis(GPIO_NUM_4);
    fb = esp_camera_fb_get();
    if(!fb){
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("Camera capture failed");
      Serial.println("Camera capture failed");
      return;
    }
    //digitalWrite(GPIO_NUM_4, LOW);
    //rtc_gpio_hold_en(GPIO_NUM_4);
    // Photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    // Insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    // Close the file
    file.close();
    esp_camera_fb_return(fb);
    digitalWrite(GPIO_NUM_4, LOW);
    rtc_gpio_hold_en(GPIO_NUM_4);
    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
  }while(!ok);
}

void initWiFi(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Connecting to WiFi...");
    Serial.println("Connecting to WiFi...");
  }
}

void initSPIFFS(){
  if(!SPIFFS.begin(true)){
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("SPIFFS failed to mount");
    Serial.println("SPIFFS failed to mount");
    ESP.restart();
  }
  else{
    delay(500);
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("SPIFFS mounted successfully");
    Serial.println("SPIFFS mounted successfully");
  }
}

void initCamera(){
 // OV2640 camera module
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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if(err != ESP_OK){
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Camera init failed");
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  } 

  //set the camera parameters
  sensor_t * s = esp_camera_sensor_get();
  s->set_reg(s, 273, 63, 5);
  //s->set_contrast(s, 2);    //min=-2, max=2
  //s->set_brightness(s, 2);  //min=-2, max=2
  //s->set_saturation(s, 2);  //min=-2, max=2
  delay(100);               //wait a little for settings to take effect
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  initLCD();
  initWiFi();
  initSPIFFS();
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  initCamera();
  pinMode(relay, OUTPUT);
  //pinMode(distanceSensor,INPUT);
  //Firebase
  // Assign the api key
  configF.api_key = API_KEY;
  //Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  /* Assign the RTDB URL (required) */
  configF.database_url = DATABASE_URL;
  /* Sign up */
  if (Firebase.signUp(&configF, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", configF.signer.signupError.message.c_str());
  }
  //Assign the callback function for the long running token generation task
  configF.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&configF, &auth);
  Firebase.reconnectWiFi(true);
}

void loop(){
  // int distanceSensorState = digitalRead(distanceSensor);

  if(takeNewPhoto){
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Taking a photo...");
    capturePhotoSaveSpiffs();
    takeNewPhoto = false;
  }
  else{
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Error: SPIFFS failure");
    Serial.println("Error: failed to capture photo and save to SPIFFS --> ");
    Serial.println(fbdo.errorReason());
    ESP.restart();
  }
  delay(1);
  
  if(Firebase.ready() && signupOK && !taskCompleted){
    taskCompleted = true;
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Uploading picture...");
    Serial.println("Uploading picture...");
    if(Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, FILE_PHOTO /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, FILE_PHOTO /* path of remote file stored in the bucket */, "image/jpeg" /* mime type */)){
      Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
      delay(2000); /////harus dicoba timingnya dengan face detection
      // do{
      //   if(Firebase.RTDB.getInt(&fbdo, "/camera")){ //getting info from firebase of camera face recognition are true
      //     if(fbdo.dataType() == "int"){
      //       intCamera = fbdo.intData();
      //       Serial.println("intCamera value = " + String(intCamera) + " --> Success");
      //       delay(200);
      //     }
      //     else{
      //       Serial.println("Error: failed to get /camera value --> ");
      //       Serial.println(fbdo.errorReason());
      //       delay(200);
      //     }
      //   }
      //   else{
      //     Serial.println("Error: RTDB.getInt /camera failed --> ");
      //     Serial.println(fbdo.errorReason());
      //     delay(200);
      //   }

      //   //////////////////////////////////////////////////////////////////////////////////////////////////////////
      //   if(Firebase.RTDB.getInt(&fbdo, "/appbutton")){ //getting info from firebase of app button are true
      //     if(fbdo.dataType() == "int"){
      //       intAppButton = fbdo.intData();
      //       Serial.println("intAppButton value = " + String(intAppButton) + " --> Success");
      //       Serial.println("");
      //       delay(200);          
      //       }
      //     else{
      //       Serial.println("Error: failed to get /appbutton value --> ");
      //       Serial.println(fbdo.errorReason());
      //       delay(200);          
      //       }
      //   }
      //   else{
      //     Serial.println("Error: RTDB.getInt /appbutton failed --> ");
      //     Serial.println(fbdo.errorReason());
      //     delay(200);
      //   }

        taskCompleted = false;
        takeNewPhoto = false;
        bool systemIdle = true;
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        do{
          if(systemIdle){
            if(intCamera == 2){  //decision making if camera failed to recognize face
              if(intCamera == 1){
                digitalWrite(relay, LOW); //current flowing
                lcd.backlight();
                lcd.setCursor(0,0);
                lcd.print("intCamera value 1");
                Serial.println("The intCamera value = " + String(intCamera) + " --> Retake Result: Face Approved! (Door UNLOCKED)");
                delay(5000);
                Firebase.RTDB.setInt(&fbdo, "/camera", intCamera = 0);
                digitalWrite(relay, HIGH);
                takeNewPhoto = false;
              }
              digitalWrite(relay, HIGH);
              lcd.backlight();
              lcd.setCursor(0,0);
              lcd.print("initCamera 1 FAILED");
              Serial.println("Failed to recognize face, please stand once again for retake the picture");
              Firebase.RTDB.getInt(&fbdo, "/camera");
              intCamera = fbdo.intData();
              Firebase.RTDB.getInt(&fbdo, "/appbutton");
              intAppButton = fbdo.intData();
              takeNewPhoto = true;
              systemIdle = false;
              program = true;
              taskCompleted = false;
              // delay(2000);
            }
            else if(intCamera == 0 && intAppButton == 0){  //decision making if both value = 0 (if there's no feedback)
              digitalWrite(relay, HIGH); //current not flowing
              lcd.backlight();
              lcd.setCursor(0,0);
              lcd.print("No interaction (Locked)");
              Serial.println("No interaction (Door LOCKED)");
              Firebase.RTDB.getInt(&fbdo, "/camera");
              intCamera = fbdo.intData();
              Firebase.RTDB.getInt(&fbdo, "/appbutton");
              intAppButton = fbdo.intData();
              takeNewPhoto = false;
              delay(200);
            }
            else if(intCamera == 1 && intAppButton == 0){ //decision making if intCamera value = 1
              digitalWrite(relay, LOW); //current flowing
              lcd.backlight();
              lcd.setCursor(0,0);
              lcd.print("Door Unlocked: FR");
              Serial.print("The intCamera value = ");
              Serial.print(intCamera);
              Serial.println(" --> Door UNLOCKED via Face Recognition");
              Firebase.RTDB.getInt(&fbdo, "/camera");
              intCamera = fbdo.intData();
              Firebase.RTDB.getInt(&fbdo, "/appbutton");
              intAppButton = fbdo.intData();
              Firebase.RTDB.setInt(&fbdo, "/camera", intCamera=0);
              delay(5000);
              digitalWrite(relay, HIGH);
              takeNewPhoto = false;
            }
            else if(intCamera == 0 && intAppButton == 1){ //decision making if intAppButton value = 1
              digitalWrite(relay, LOW);
              lcd.backlight();
              lcd.setCursor(0,0);
              lcd.print("Door Unlocked: AB");
              Serial.print("The intAppButton value = ");
              Serial.print(intAppButton);
              Serial.println(" --> Door UNLOCKED via App Button");
              Firebase.RTDB.getInt(&fbdo, "/camera");
              intCamera = fbdo.intData();
              Firebase.RTDB.getInt(&fbdo, "/appbutton");
              intAppButton = fbdo.intData();
              Firebase.RTDB.setInt(&fbdo, "/appbutton", intAppButton=0);
              delay(5000);
              digitalWrite(relay, HIGH);
              takeNewPhoto = false;
            }
            else if(intCamera == 1 && intAppButton == 1){ //decision making if both value = 1
              digitalWrite(relay, LOW);
              lcd.backlight();
              lcd.setCursor(0,0);
              lcd.print("Door Unlocked: Both");
              Serial.println(" --> Door UNLOCKED via both Camera and App Button");
              Firebase.RTDB.getInt(&fbdo, "/camera");
              intCamera = fbdo.intData();
              Firebase.RTDB.getInt(&fbdo, "/appbutton");
              intAppButton = fbdo.intData();
              Firebase.RTDB.setInt(&fbdo, "/camera", intCamera=0);
              Firebase.RTDB.setInt(&fbdo, "/appbutton", intAppButton=0);
              delay(5000);
              digitalWrite(relay, HIGH);
              takeNewPhoto = false;
            }
            else{
              lcd.backlight();
              lcd.setCursor(0,0);
              lcd.print("Error: value failed");
              Serial.println("Error: failed to get the value --> " + fbdo.errorReason() + " --> Restarting ESP32-CAM");
              //Serial.println(fbdo.errorReason());
              delay(1000);
              ESP.restart();
            }
          }
        }while(systemIdle);
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
      // }while(!program);
    }
    else{
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("Error after do-while loop");
      Serial.println("Error after do-while loop --> " + fbdo.errorReason() + " --> Restarting ESP32-CAM");
      //Serial.println(fbdo.errorReason());
      delay(1000);
      ESP.restart();
    }
  }
  else{
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Error on firebase.ready");
    Serial.println("Error on firebase.ready --> " + fbdo.errorReason() + " --> Restarting ESP32-CAM");
    //Serial.println(fbdo.errorReason());
    delay(1000);
    ESP.restart();
  }
}
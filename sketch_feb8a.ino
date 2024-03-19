#include <SPI.h>
#define CSad 10 //Chip select
//#include <TFT.h>  
/*
#define TFT_CS     6
#define TFT_RST    9
#define TFT_DC     8


TFT tft = TFT(TFT_CS, TFT_DC, TFT_RST);
*/
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int redPin = 6;    // Nowy pin diody czerwonej
const int greenPin = 5;  // Nowy pin diody zielonej
const int bluePin = 3;   // Nowy pin diody niebieskiej


uint8_t val16;
uint8_t receivedVal;
uint8_t kom=0x60;
uint16_t itemp;/*internal temperature*/
uint16_t valit;
uint8_t konf_temp=0b00010000;
uint16_t sel_temp=0b0000000010000110;//0b00001000000010110
uint8_t czyt_temp=0b01011100;

uint16_t value; 
uint16_t value2; 
uint16_t  Code;
uint16_t Code2;
float internaltep;
float AIN;

uint16_t full_Scale_Calibration=0b11001000000001010;//11111000000001010
uint16_t zero_Scale_Calibration=0b11101000000001010;

uint8_t adress_mode_register=0b00001000; //01 011 100
uint8_t adress_config_register=0b00010000;//01011000
uint8_t adress_data_register=0b01011100;//było 0b00011000
uint8_t adress_data_register_single=0b01011000;//było 0b00011000
uint8_t adress_offset_register=0b01110000;
uint8_t adress_full_scale_register=0b01111000;

uint8_t data_register_reset=0x0000;
uint16_t mode_register=0b0000000000000001;//ma być 0b0000000000000001;
uint16_t mode_register_single_conversation=0b0010000000000001;//ma być 0b0000000000000001;
uint16_t config_register=0b0000000000000000;//0b0001000000000000;uniporal 0b0000000000000000;biporal
//uint16_t config_register=0b0000000010000000; intrnal
uint16_t config_register_unipolar_temp=0b00001000000000110;
//uint16_t sel_temp=0b00001000000010110;//unipolar
uint16_t idle_register=0b0100000000000001;
uint16_t power_down_register=0b001100000000001;
uint16_t data_register;
uint16_t offset_register;
uint16_t full_scale_register;
uint16_t offset_register_1;
uint16_t full_scale_register_1;



void id() {
  digitalWrite(CSad, LOW);
  SPI.transfer(0x8);
  SPI.transfer16(0x500A);
  digitalWrite(CSad, HIGH);
  delay(5);
  digitalWrite(CSad, LOW);
  SPI.transfer(kom);
  delay(3);
  uint8_t receivedVal = SPI.transfer(val16);
  Serial.print('\n');
  Serial.print("id to: ");
  Serial.print(receivedVal, HEX);
  Serial.print('\n');
  delay(100);
  digitalWrite(CSad, HIGH);
  delay(100);
}

void internal_temperature() {
    digitalWrite(CSad, LOW);
    SPI.transfer16(0xFFFF);
    SPI.transfer16(0xFFFF);
    digitalWrite(CSad, HIGH); // Assuming it's HIGH rather than 1

    delay(10);
    digitalWrite(CSad, LOW);
    SPI.transfer(0x8);
    SPI.transfer16(0x100A);
    digitalWrite(CSad, HIGH); // Assuming it's HIGH rather than 1
    digitalWrite(CSad, LOW);
    SPI.transfer(konf_temp);
    SPI.transfer16(sel_temp);
    digitalWrite(CSad, HIGH); // Assuming it's HIGH rather than 1
    digitalWrite(CSad, LOW);
    SPI.transfer(adress_data_register_single);

    do {
       // Serial.print("szczeka temp");
        //Serial.print('\t');
    } while (digitalRead(MISO) == HIGH);

    uint16_t Code = SPI.transfer16(valit);
    Serial.print("Code to: ");
    Serial.print(Code, HEX);
    Serial.print('\n');

    float AIN = (Code - 32768) * 1.17 / ((pow(2, 15) - 1) * 1);
    float internaltep = ((AIN / 0.00081) - 273.15);
    Serial.println("Temperature:");
    Serial.println(internaltep, 2);

    delay(1000);
    digitalWrite(CSad, HIGH); // Assuming it's HIGH rather than 1
}

void full_Scale_Calibration_function() {
    digitalWrite(CSad, LOW);
    while (digitalRead(MISO) == LOW);
    SPI.transfer(adress_mode_register);
    delay(100);
    SPI.transfer16(full_Scale_Calibration);
    delay(100);
    Serial.println("full_Scale_Calibration");
    digitalWrite(CSad, HIGH);
    delay(100);
    digitalWrite(CSad, LOW);
    digitalWrite(CSad, HIGH);
}
void offset_register_and_full_scale_function() {
    digitalWrite(CSad, LOW);

    // Configuration for data register
    SPI.transfer(adress_data_register_single);
    SPI.transfer16(mode_register_single_conversation);
    digitalWrite(CSad, HIGH);

    // Configuration for config register
    digitalWrite(CSad, LOW);
    SPI.transfer(adress_config_register);
    SPI.transfer16(config_register);
    digitalWrite(CSad, HIGH);

    // Configuration for offset register
    digitalWrite(CSad, LOW);
    SPI.transfer(adress_offset_register);
    do{}
    while (digitalRead(MISO) == HIGH); // Wait for MISO to go low
    uint16_t offset_register_1 = SPI.transfer16(offset_register);
    Serial.print("offset_register ");
    Serial.print('\t');
    Serial.print(offset_register_1, HEX);
    Serial.print('\n');
    digitalWrite(CSad, HIGH);
    digitalWrite (CSad,LOW);
SPI.transfer(adress_full_scale_register);
do
{
} while (digitalRead(MISO)==1);

full_scale_register_1= SPI.transfer16(full_scale_register);
Serial.print("full_scale_register ");
Serial.print('\t');
Serial.print(full_scale_register_1,HEX);
Serial.print('\n');
}

void full_scale_register_function() {//not working idk why
    digitalWrite(CSad, LOW);

    // Configuration for data register
    SPI.transfer(adress_data_register_single);
    SPI.transfer16(mode_register_single_conversation);
    digitalWrite(CSad, HIGH);

    // Configuration for config register
    digitalWrite(CSad, LOW);
    SPI.transfer(adress_config_register);
    SPI.transfer(adress_full_scale_register);
    do{}
    while (digitalRead(MISO) == HIGH); // Wait for MISO to go low
    uint16_t full_scale_register_1 = SPI.transfer16(full_scale_register);
    Serial.print("full_scale_register ");
    Serial.print('\t');
    Serial.print(full_scale_register_1, HEX);
    Serial.print('\n');
    digitalWrite(CSad, HIGH);
}
float volt_ch1() {
    digitalWrite(CSad, LOW);

    // Configuration for data register
    SPI.transfer(adress_data_register_single);
    SPI.transfer16(mode_register_single_conversation);
    digitalWrite(CSad, HIGH);

    // Configuration for config register
    digitalWrite(CSad, LOW);
    SPI.transfer(adress_config_register);
    SPI.transfer16(config_register);
    digitalWrite(CSad, HIGH);

    // Read voltage from data register
    digitalWrite(CSad, LOW);
    SPI.transfer(adress_data_register);

    do {
        //Serial.print("czeka wolt");
        //Serial.print('\t');
    } while (digitalRead(MISO) == HIGH);

    uint16_t value = SPI.transfer16(data_register);
    Serial.print("value voltage");
    Serial.print('\t');
    Serial.print(value, HEX);
    Serial.print('\t');
    Serial.print(" ; Voltage[mV]=");
    double voltage = ((float(value) - 32768) / (pow(2, 15) - 1) * 5)*1000; //Output in Voltage biporal
    Serial.print(voltage,4);
    Serial.print('\n');
    digitalWrite(CSad, HIGH);
    return voltage;
}
double checkMaterialSafety() {
    float voltage = volt_ch1();
    float tensometer_resistance = 119.6;
    int reference_voltage = 5;
    float k = 2.15; // Stała tensometru
    int E = 70000; // Young module 7*10^9GPa -ALumminum but dividet to MPa

    double resistanceChange = tensometer_resistance * (voltage /1000* reference_voltage);
    double sigma = E * resistanceChange / (tensometer_resistance * k);
    double epsilon = sigma/E;
    double force=sigma*0.00000589*1000000;
    sigma = abs(sigma);
    Serial.println("Naprezenia MPa : ");
    Serial.println(sigma,10);
    Serial.println("odksztalcenie : ");
    Serial.println(epsilon,10);

    Serial.print('\n');

    const double tensileStrength = 500; // Przykładowe naprężenie krytyczne na zrywanie w MPa
    const double compressiveStrength = 400; // Przykładowe naprężenie krytyczne na ściskanie w MPa
    const double bendingStrength = 300; // Przykładowe naprężenie krytyczne na zginanie w MPa

    // Sprawdzanie warunków naprężenia
    if (sigma < tensileStrength) {
        Serial.println("Applied tensile stress is below tensile strength. Material is safe under tension.");
    } else {
        Serial.println("Applied tensile stress exceeds tensile strength. Material is at risk of failure under tension.");
    }

    if (sigma < compressiveStrength) {
        Serial.println("Applied compressive stress is below compressive strength. Material is safe under compression.");
    } else {
        Serial.println("Applied compressive stress exceeds compressive strength. Material is at risk of failure under compression.");
    }

    if (sigma < bendingStrength) {
        Serial.println("Applied bending stress is below bending strength. Material is safe under bending.");
    } else {
        Serial.println("Applied bending stress exceeds bending strength. Material is at risk of failure under bending.");
    }

    displayOnLCD(sigma);
    //displayOnLCD(force);
    return sigma;
}
/*
void displayMaterialSafety(double sigma, double tensileStrength, double compressiveStrength, double bendingStrength) {
    tft.setTextSize(2);
    tft.stroke(255, 255, 255);
    tft.text("Tensions : ", 0, 0);

    char sigma_str[10];
    sprintf(sigma_str, "%.2f", sigma);
    tft.text(sigma_str, 0, 20);
    tft.text("\n", 0, 40);

    if (sigma < 20) {
        tft.background(0, 255, 0); // Zielone tło
    } else if (sigma < 10) {
        tft.background(255, 255, 0); // Żółte tło
    } else {
        tft.background(255, 0, 0); // Czerwone tło
    }

    if (sigma < tensileStrength) {
        tft.text("Applied tensile stress is below tensile strength. Material is safe under tension.\n", 0, 60);
    } else {
        tft.text("Applied tensile stress exceeds tensile strength. Material is at risk of failure under tension.\n", 0, 60);
    }

    // Analogiczne sprawdzenie dla naprężenia krytycznego na ściskanie
    if (sigma < compressiveStrength) {
        tft.text("Applied compressive stress is below compressive strength. Material is safe under compression.\n", 0, 100);
    } else {
        tft.text("Applied compressive stress exceeds compressive strength. Material is at risk of failure under compression.\n", 0, 100);
    }

    // Analogiczne sprawdzenie dla naprężenia krytycznego na zginanie
    if (sigma < bendingStrength) {
        tft.text("Applied bending stress is below bending strength. Material is safe under bending.\n", 0, 140);
    } else {
        tft.text("Applied bending stress exceeds bending strength. Material is at risk of failure under bending.\n", 0, 140);
    }
}
*/






// Example of calling the function



void displayOnLCD(double sigma) {
    // Konwersja wartości sigma na postać tekstową
    char sigmaStr[10];
    char forceStr[10]; // Zakładamy, że wartość sigma będzie mieścić się w 10 znakach
    dtostrf(sigma, 5, 3, sigmaStr);
    //dtostrf(force, 5, 3, forceStr); // Konwersja liczby na 5 cyfr, z czego 2 po przecinku

    // Wyświetlenie wartości sigma na LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Tension:"); lcd.print(sigmaStr);lcd.print("MPa");
    lcd.setCursor(0, 1);
    
    //lcd.print("F: "); lcd.print(forceStr);lcd.print("N");

}


void controlRGB(double sigma) {
   if (sigma <= 5) { // Jeśli sigma jest mniejsza lub równa 5
    analogWrite(redPin, 0);   // Wyłącz czerwoną diodę
    analogWrite(greenPin, 0); // Wyłącz zieloną diodę
    analogWrite(bluePin, 0);  // Wyłącz niebieską diodę
   }
  else if (sigma <= 10) { // Jeśli sigma jest mniejsza lub równa 10
    analogWrite(redPin, 0);   // Wyłącz czerwoną diodę
    analogWrite(greenPin, 255); // Włącz zieloną diodę
    analogWrite(bluePin, 0);  // Wyłącz niebieską diodę
  } else if (sigma > 10 && sigma <= 20) { // Jeśli sigma jest większa niż 10 i mniejsza lub równa 20
    analogWrite(redPin, 0);   // Wyłącz czerwoną diodę
    analogWrite(greenPin, 0);  // Wyłącz zieloną diodę
    analogWrite(bluePin, 255); // Włącz niebieską diodę
  } else { // W przeciwnym razie (jeśli sigma jest większa niż 20)
    analogWrite(redPin, 255); // Włącz czerwoną diodę
    analogWrite(greenPin, 0);  // Wyłącz zieloną diodę
    analogWrite(bluePin, 0);   // Wyłącz niebieską diodę
  }
}
void setup() {
  // put your setup code here, to run once:
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
   Serial.begin(2400,SERIAL_8E2);
  SPI.begin();
 
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  digitalWrite(CSad, LOW);
  SPI.transfer16(0xFFFF);
  SPI.transfer16(0xFFFF);
  digitalWrite(CSad,1);
  id();
  lcd.init();       // Inicjalizacja LCD
  lcd.backlight();  // Włączenie podświetlenia


}

void loop() {
  // put your main code here, to run repeatedly:

full_Scale_Calibration_function();
internal_temperature();
offset_register_and_full_scale_function();
checkMaterialSafety();
double sigma = checkMaterialSafety();
controlRGB(sigma);

}

#include <Wire.h>
#include <Adafruit_INA219.h>

// Inisialisasi sensor INA219 dengan alamat I2C yang berbeda
Adafruit_INA219 ina219_1(0x40);  // Sensor pertama, alamat I2C default
Adafruit_INA219 ina219_2(0x41);  // Sensor kedua, alamat I2C yang diubah

// Kalman Filter untuk Tegangan dan Arus
class Kalman {
public:
    Kalman() {
        Q = 0.1f;  // Proses noise covariance
        R = 0.1f;  // Measurement noise covariance
        P = 1.0f;  // Error covariance
        K = 0.0f;  // Kalman gain
        X = 0.0f;  // Estimasi awal
    }

    float getEstimate(float measurement) {
        // Prediksi
        P = P + Q;

        // Kalman Gain
        K = P / (P + R);

        // Update
        X = X + K * (measurement - X);

        // Update error covariance
        P = (1 - K) * P;

        return X;
    }

private:
    float Q, R, P, K, X;
};

// Inisialisasi Kalman Filter untuk tegangan dan arus
Kalman kalmanVoltage1;
Kalman kalmanCurrent1;
Kalman kalmanVoltage2;
Kalman kalmanCurrent2;

const int controlPin1 = 27;  // Pin kontrol pengisian untuk sensor 1 (HIGH/LOW)
const int controlPin2 = 25;  // Pin kontrol pengisian untuk sensor 2 (HIGH/LOW)

void setup() {
    // Memulai komunikasi serial
    Serial.begin(115200);

    // Mengatur pin kontrol sebagai output
    pinMode(controlPin1, OUTPUT);
    pinMode(controlPin2, OUTPUT);

    // Inisialisasi INA219 pertama
    if (!ina219_1.begin()) {
        Serial.println("Gagal menemukan INA219_1!");
        while (1);  // Jika sensor tidak ditemukan, berhenti di sini
    }

    // Inisialisasi INA219 kedua
    if (!ina219_2.begin()) {
        Serial.println("Gagal menemukan INA219_2!");
        while (1);  // Jika sensor tidak ditemukan, berhenti di sini
    }

    Serial.println("INA219 siap!");
    Serial.println("Masukkan 'HIGH' atau 'LOW' untuk mengontrol pengisian (sensor 1 atau sensor 2):");
}

void loop() {
    // Mengecek input dari Serial Monitor untuk kontrol charging pada sensor 1
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');  // Membaca input hingga newline

        if (input == "1") {
            digitalWrite(controlPin1, HIGH);  // Mengaktifkan pengisian untuk sensor 1
            Serial.println("Pengisian Sensor 1 Aktif");
        } else if (input == "2") {
            digitalWrite(controlPin1, LOW);  // Memutuskan pengisian untuk sensor 1
            Serial.println("Pengisian Sensor 1 Diputuskan");
        } else if (input == "3") {
            digitalWrite(controlPin2, HIGH);  // Mengaktifkan pengisian untuk sensor 2
            Serial.println("Pengisian Sensor 2 Aktif");
        } else if (input == "4") {
            digitalWrite(controlPin2, LOW);  // Memutuskan pengisian untuk sensor 2
            Serial.println("Pengisian Sensor 2 Diputuskan");
        } else {
            Serial.println("Input tidak valid. Masukkan 'HIGH', 'LOW', 'HIGH2' atau 'LOW2'");
        }
    }

    // Membaca tegangan dan arus dari INA219 pertama
    float busVoltage1 = ina219_1.getBusVoltage_V();
    float current1 = ina219_1.getCurrent_mA();

    // Membaca tegangan dan arus dari INA219 kedua
    float busVoltage2 = ina219_2.getBusVoltage_V();
    float current2 = ina219_2.getCurrent_mA();

    // Menyaring nilai tegangan dan arus dengan Kalman Filter untuk sensor 1
    float filteredVoltage1 = kalmanVoltage1.getEstimate(busVoltage1);
    float filteredCurrent1 = kalmanCurrent1.getEstimate(current1);

    // Menyaring nilai tegangan dan arus dengan Kalman Filter untuk sensor 2
    float filteredVoltage2 = kalmanVoltage2.getEstimate(busVoltage2);
    float filteredCurrent2 = kalmanCurrent2.getEstimate(current2);

    // Menampilkan tegangan dan arus yang sudah difilter di Serial Monitor
    Serial.print("Tegangan Baterai Sensor 1: ");
    Serial.print(filteredVoltage1);
    Serial.println(" V");

    Serial.print("Arus Pengisian Sensor 1: ");
    Serial.print(filteredCurrent1);
    Serial.println(" mA");

    Serial.print("Tegangan Baterai Sensor 2: ");
    Serial.print(filteredVoltage2);
    Serial.println(" V");

    Serial.print("Arus Pengisian Sensor 2: ");
    Serial.print(filteredCurrent2);
    Serial.println(" mA");

    // Menentukan apakah pengisian aktif atau tidak untuk sensor 1
    if (filteredCurrent1 > 0) {
        Serial.println("Pengisian Sensor 1 Berlangsung");
    } else {
        Serial.println("Tidak ada Pengisian Sensor 1");
    }

    // Menentukan apakah pengisian aktif atau tidak untuk sensor 2
    if (filteredCurrent2 > 0) {
        Serial.println("Pengisian Sensor 2 Berlangsung");
    } else {
        Serial.println("Tidak ada Pengisian Sensor 2");
    }

    // Memberikan jeda 1 detik sebelum pembacaan berikutnya
    delay(1000);
}

#include "mbed.h"
#include "MPU9250.h"

#define BUFFER_SIZE     50
#define MAX_VCC          3.3
#define RESOLUTION       65531.0
#define CSCA_SENSITIVITY 0.010

InterruptIn sensor1(PB_1, PullNone);
InterruptIn sensor2(PA_4, PullNone);
AnalogIn sens_pin(PA_1);

MPU9250 mpu9250;

void ticker5HzISR();
void ticker20HzISR();
void ticker2HzISR();
void ticker50HzISR();

void setupInterrupts();

void contadorFrequenciaISR1();
void contadorFrequenciaISR2();

// ---Global variables
static int points = 500;
static float adc_to_current = MAX_VCC / (RESOLUTION * CSCA_SENSITIVITY);

int get_offset(void)
{
    long avg = 0;

    for (int i = 0; i < points; i++) {
        avg += sens_pin.read_u16();
    }

    return round(avg/points);
}

Ticker ticker5Hz;
Ticker ticker20Hz;
Ticker ticker2Hz;
Ticker ticker50Hz;
CircularBuffer <int, BUFFER_SIZE> state_buffer;
uint8_t switch_state = 0x00;
int current_state = 0;

Timer t;
uint8_t contagem_pulso1 = 0, contagem_pulso2 = 0;
uint64_t periodo_atual1 = 0, ultimo_t1 = 0, periodo_atual2 = 0, ultimo_t2 = 0;
float f1 = 0;
float f2 = 0;

int main() {
    i2c.frequency(400000);  // Inicializa um I2C rápido (400Hz)  
  
    t.start();

    mpu9250.resetMPU9250(); // Reinicia os registradores para o default para preparar para a calibração do dispositivo
    mpu9250.MPU9250SelfTest(SelfTest); // Começa realizando um teste inicial e reporta os valores
    mpu9250.initMPU9250(); // Inicializa o MPU

    mpu9250.getAres(); // Sensitividade do acelerômetro
    mpu9250.getGres(); // Sensitividade do giroscópio

    int adc_value;
    int offset = get_offset();
    float current_value;

    printf("\n\t---Sensor CSCA400---\n");

    setupInterrupts(); 

    t.start();

    while (true) {
        if (state_buffer.full()) {
            state_buffer.pop(current_state);
        }
        else {
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = 0;
        }
        switch (current_state) {
            case 0:
                sensor1.fall(NULL); // Desativa o interrupt
                if (periodo_atual1 != 0){
                    f1 = 1000000 * ((float)(contagem_pulso1)/periodo_atual1); // Calcula a frequência em Hz
                }
                else{
                    f1 = 0;
                }

                // Reinicialização dos parâmetro do pulso
                contagem_pulso1 = 0;                          
                periodo_atual1 = 0;                  
                ultimo_t1 = t.elapsed_time().count();        
                sensor1.fall(&contadorFrequenciaISR1); // Ativa o interrupt

                printf("%d\n", int(f1 * 30)); // Converte a velocidade para RPM
                break;
            case 1:
                if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // Na interrupção, checa se a data está prota para interromper
                
                    mpu9250.readAccelData(accelCount);  // Lê os valores x/y/z do adc   
                    // Calcula a aceleração em g's
                    ax = (float)accelCount[0] * aRes - accelBias[0];
                    ay = (float)accelCount[1] * aRes - accelBias[1];   
                    az = (float)accelCount[2] * aRes - accelBias[2];  
                
                    mpu9250.readGyroData(gyroCount);  // Lê os valores x/y/z do adc  
                    // Calcula a velocidade angular em graus por segundo
                    gx = (float)gyroCount[0] * gRes - gyroBias[0];
                    gy = (float)gyroCount[1] * gRes - gyroBias[1];  
                    gz = (float)gyroCount[2] * gRes - gyroBias[2];   
                }

                // Normalização e conversão dos valores obtidos
                mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);

                // Printa os dados de aceleração convertendo para m/s² e fazendo a conversão
                printf("ax = %f", ax * 9.81 - 0.15); 
                printf(" ay = %f", ay * 9.81 - 0.1); 
                printf(" az = %f  m/s²\n\r", az * 9.81 + 0.12);

                // Printa os dados de velocidade angular
                printf("gx = %f", gx); 
                printf(" gy = %f", gy); 
                printf(" gz = %f  rad/s\n\r", gz); 

                // Lê o valores adc de temperatura em Kelvin, converte para graus Celsius e printa
                tempCount = mpu9250.readTempData();
                temperature = ((float) tempCount) / 333.87f + 21.0f;
                printf(" temperature = %f  C\n\r", temperature); 
                break;
                case 2:
                sensor2.fall(NULL); // Desativa o interrupt
                if (periodo_atual2 != 0){
                    f2 = 1000000 * ((float)(contagem_pulso2)/periodo_atual2); // Calcula a frequência em Hz
                }
                else{
                    f2 = 0;
                }

                // Reinicialização dos parâmetro do pulso
                contagem_pulso2 = 0;                          
                periodo_atual2 = 0;                  
                ultimo_t2 = t.elapsed_time().count();        
                sensor2.fall(&contadorFrequenciaISR2); // Ativa o interrupt

                printf("%d\n", int(f2 * 30)); // Converte a velocidade para RPM
                break;
                case 3:
                current_value = abs(sens_pin - offset) * adc_to_current;
                printf("%f\n", current_value);
                break;
        }
    }
}

void ticker5HzISR() {
    state_buffer.push(0);
}

void ticker20HzISR() {
    state_buffer.push(1);
}

void ticker2HzISR() {
    state_buffer.push(2);
}

void ticker50HzISR() {
    state_buffer.push(3);
}


void setupInterrupts() {
    ticker5Hz.attach(&ticker5HzISR, 200ms);
    ticker20Hz.attach(&ticker20HzISR, 50ms);
    ticker2Hz.attach(&ticker2HzISR, 500ms);
    ticker50Hz.attach(&ticker50HzISR, 20ms);
}

void contadorFrequenciaISR1(){
    contagem_pulso1++;                                     // Quantidade de pulsos que aconteceram
    periodo_atual1 += t.elapsed_time().count() - ultimo_t1; // Tamanho do pulso em s
    ultimo_t1 = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}

void contadorFrequenciaISR2(){
    contagem_pulso2++;                                     // Quantidade de pulsos que aconteceram
    periodo_atual2 += t.elapsed_time().count() - ultimo_t2; // Tamanho do pulso em s
    ultimo_t2 = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}

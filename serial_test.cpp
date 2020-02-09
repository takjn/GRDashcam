// #include "mbed.h"
// #include "USBHostSerialCustom.h"

// DigitalOut led(LED1);
// Serial pc(USBTX, USBRX);
// USBHostSerialCustom serial(500);

// void serial_task(void const*) {
    
//     while(1) {
    
//         pc.printf("connecting\r\n");
//         // try to connect a serial device
//         while (!serial.connect()) {
//             Thread::wait(500);
//             pc.printf(".");
//         }
//         serial.baud(38400);

//         pc.printf("connected\r\n");

//         serial.printf("ATI\r");
//         Thread::wait(1000);

//         serial.printf("ATL1\r");
//         Thread::wait(1000);

//         // serial.printf("0100\r");
//         // Thread::wait(1000);

//         // serial.printf("atrv\r");
//         // Thread::wait(1000);

//         // in a loop, print all characters received
//         // if the device is disconnected, we try to connect it again
//         while (1) {
//             // if device disconnected, try to connect it again
//             if (!serial.connected()) {
//                 break;
//             }

//             // print characters received
//             while (serial.available()) {
//                 char c = serial.getc();
//                 pc.putc(c);
//             }

//             if (pc.readable()) {
//                 char c = pc.getc();
//                 serial.putc(c); 
//             }
//         }
//     }
// }

// int main() {
//     pc.baud(115200);
//     pc.printf("Hello World!\n\r");

//     serial_task("test");
//     // Thread serialTask(serial_task, NULL, osPriorityNormal, 256 * 32);
//     // while(1) {
//     //     led=!led;
//     //     Thread::wait(500);
//     // }
// }
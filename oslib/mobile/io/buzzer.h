/* scu_ble.h
 * Created: 28/03/2022
 * Updated: 1/04/2022
 * Author: Matthew Kumar
 */
#ifndef BUZZER_H
#define BUZZER_H

// struct for the message queue for buzzer
struct buzzer_freq {
    int freq;
};

// the message queue variable
extern struct k_msgq buzzer_queue;

void buzzer_init();

#endif
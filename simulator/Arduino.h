#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <stdint.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <iostream>

// Types Arduino
typedef uint8_t byte;
typedef bool boolean;

// Constantes
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1

// Mock des registres
extern volatile uint8_t MOCK_PORTB;
extern volatile uint8_t MOCK_PORTD;

// Mock des fonctions de temps
// On utilise steady_clock pour simuler micros()
extern std::chrono::time_point<std::chrono::steady_clock> start_time;

static inline unsigned long micros() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time);
    return (unsigned long)duration.count();
}

static inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static inline void delayMicroseconds(unsigned int us) {
    // Active busy-wait for precision in simulation (sleep_for is too inaccurate on Windows)
    auto start = std::chrono::steady_clock::now();
    while(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count() < us);
}

// Mock des IO
inline void pinMode(uint8_t pin, uint8_t mode) {
    // No-op for simulation
}

// Simulation simple : 
// Pin 0-7 = PORTD
// Pin 8-13 = PORTB
// On stocke l'état des pins dans un tableau global pour le loopback
extern uint8_t pin_states[20];

inline void digitalWrite(uint8_t pin, uint8_t val) {
    pin_states[pin] = val;
    // Mise à jour des registres mockés pour la compatibilité avec le code optimisé
    if (pin >= 0 && pin <= 7) {
        if (val) MOCK_PORTD |= (1 << pin);
        else     MOCK_PORTD &= ~(1 << pin);
    } else if (pin >= 8 && pin <= 13) {
        if (val) MOCK_PORTB |= (1 << (pin - 8));
        else     MOCK_PORTB &= ~(1 << (pin - 8));
    }
}

inline int digitalRead(uint8_t pin) {
    // Read from registers to support direct port manipulation
    if (pin >= 0 && pin <= 7) return (MOCK_PORTD & (1 << pin)) ? HIGH : LOW;
    if (pin >= 8 && pin <= 13) return (MOCK_PORTB & (1 << (pin - 8))) ? HIGH : LOW;
    return pin_states[pin];
}

// Mocks avancés pour l'optimisation AVR
#define digitalPinToPort(P) (P < 8 ? 1 : 2) // 1=D, 2=B
#define digitalPinToBitMask(P) (P < 8 ? (1 << P) : (1 << (P-8)))

inline volatile uint8_t* portOutputRegister(uint8_t port) {
    if (port == 1) return &MOCK_PORTD;
    return &MOCK_PORTB;
}

// Interrupts
inline void noInterrupts() {}
inline void interrupts() {}

#endif

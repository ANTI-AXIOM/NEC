/*************************************************************************************************************************************/
// Sujet   :  Definition de la bibliotheque de fonctions NEC                                                                         //
// Auteurs :  AUGEREAU F. JOBARD L.                                                                                                  //
// Date    :  14/03/2025                                                                                                             //
// Version :  1.5.3                                                                                                                  //
/*************************************************************************************************************************************/
//                                                                                                                                   //
// 2 fonctions sont fournies dans cette bibliotheque NEC :                                                                           //
//                                                                                                                                   //
//   => fonction d'emission d'une trame NEC                                                                                          //
//     syntaxe : void GenererTrameNEC(int Broche, uint8_t Adresse, uint8_t Donnee);                                                  //
//          Broche est la valeur de la broche Arduino sur laquelle est connectee la LED d'émission infrarouge                        //
//          Adresse est la valeur de l'adresse NEC à transmettre dans la trame NEC                                                   //
//          Donnee est la valeur de la donnee NEC à transmettre dans la trame NEC                                                    //
//                                                                                                                                   //
//   => fonction de reception d'une trame NEC                                                                                        //
//     syntaxe : int8_t AcquerirTrameNEC(int Broche, uint8_t* Adresse, uint8_t* Donnee);                                             //
//          AcquerirTrameNEC retourne une valeur d'erreur (0 : trame NEC valide ; -1 : erreur de reception NEC ; -2 : trame absente) //
//          Broche est la valeur de la broche Arduino sur laquelle est connectee le recepteur infrarouge                             //
//          *Adresse retourne la valeur de l'adresse NEC incluse dans la trame NEC recue (passage de la valeur par pointeur)         //
//          *Donnee retourne la valeur de la donnee NEC incluse dans la trame NEC recue (passage de la valeur par pointeur)          //
//                                                                                                                                   //
/*************************************************************************************************************************************/

#include "NEC.h"

// =================================================================================
// Utilitaires ASM & Timing
// =================================================================================

// Délai précis en assembleur (pour AVR 16MHz)
// 1 cycle = 1/16 µs = 62.5ns
// La boucle prend 4 cycles.
static inline void asmDelay(uint32_t cycles) {
    if (cycles == 0) return;
#ifdef __AVR__
    __asm__ __volatile__ (
        "1: sbiw %0, 1\n\t" // 2 cycles
        "brne 1b"           // 2 cycles (sauf dernier passage 1 cycle)
        : "=w" (cycles)
        : "0" (cycles)
    );
#else
    // Fallback simulation (approximatif)
    // Simple boucle volatile pour éviter l'overhead de chrono
    // Calibrage: multiplier par une constante
    // 50 -> 1.6ms.
    // 280 -> 1.7ms (optimisé ?)
    // Essai avec NOP et facteur 2000
    volatile uint32_t count = cycles * 2000; 
    while (count--) {
        __asm__ __volatile__("nop");
    }
#endif
}

// =================================================================================
// Classe NecTx (Emission)
// =================================================================================

NecTx::NecTx(uint8_t pin) : _pin(pin), _portReg(nullptr), _pinMask(0) {}

void NecTx::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    // Port Caching : On récupère l'adresse du registre PORT et le masque binaire
    // Cela évite l'appel coûteux à digitalPinToPort/digitalWrite dans la boucle 38kHz
    uint8_t port = digitalPinToPort(_pin);
    _pinMask = digitalPinToBitMask(_pin);
    _portReg = portOutputRegister(port);
}

void NecTx::sendBurst(uint16_t durationMicroseconds) {
    // Calcul du nombre de cycles 38kHz (Période ~26.3µs)
    // 1000 µs / 26.3 µs ~= 38 cycles par ms environ
    // Plus précisément : duration / 26.315...
    uint32_t pulses = (durationMicroseconds * 38) / 1000;
    
    // Debug Macros
#ifdef NEC_DEBUG
    #include <stdio.h>
    #define NEC_LOG(...) do { printf("[NEC] " __VA_ARGS__); printf("\n"); fflush(stdout); } while(0)
#else
    #define NEC_LOG(...)
#endif
    // Debug timing for simulation
    #ifdef NEC_DEBUG
    uint32_t start = micros();
    NEC_LOG("sendBurst requested: %u us, pulses: %lu", durationMicroseconds, pulses);
    #endif

    // Pour 38kHz, période = 26µs. 
    // On doit faire : ON (~8µs) -> OFF (~18µs) (Duty cycle 1/3 souvent utilisé, ou 50%)
    // Ici on vise ~50% pour simplifier ou rester proche de l'original.
    
    // Optimisation CRITIQUE : Désactivation des interruptions pour timing parfait
    // Attention : CLI / SEI
    
    while (pulses-- > 0) {
        // ON
        *_portReg |= _pinMask; 
        asmDelay(35); // ~9µs
        
        // OFF
        *_portReg &= ~_pinMask;
        asmDelay(70); // ~17.5µs
    }
    
    #ifdef NEC_DEBUG
    uint32_t elapsed = micros() - start;
    NEC_LOG("sendBurst(%u) took %lu us", durationMicroseconds, (unsigned long)elapsed);
    #endif
}

void NecTx::sendSpace(uint16_t durationMicroseconds) {
    // Espace = silence (LED éteinte)
    *_portReg &= ~_pinMask;
    // On utilise delayMicroseconds pour les temps longs, c'est précis.
    // Mais pour garantir la stabilité, on reste en mode "interruptions désactivées" si appelé depuis send().
    if (durationMicroseconds > 0) {
        delayMicroseconds(durationMicroseconds);
    }
}

void NecTx::send(uint8_t address, uint8_t command) {
    // Préparation de la trame 32 bits : Address, ~Address, Command, ~Command
    uint32_t data = ((uint32_t)address << 24) |
                    ((uint32_t)(~address & 0xFF) << 16) |
                    ((uint32_t)command << 8) |
                    ((uint32_t)(~command & 0xFF));

    // SECTION CRITIQUE : Désactivation des interruptions
    noInterrupts();

    // 1. Header
    sendBurst(NecProtocol::HDR_MARK);
    sendSpace(NecProtocol::HDR_SPACE);

    // 2. Data
    for (int i = 31; i >= 0; i--) {
        sendBurst(NecProtocol::BIT_MARK);
        if (data & (1UL << i)) {
            sendSpace(NecProtocol::ONE_SPACE);
        } else {
            sendSpace(NecProtocol::ZERO_SPACE);
        }
    }

    // 3. Bit de fin
    sendBurst(NecProtocol::BIT_MARK);
    
    // Réactivation des interruptions
    interrupts();
    
    // Délai de garde après transmission pour séparer les commandes
    // Fait avec interruptions actives pour ne pas bloquer millis() trop longtemps
    delay(40); 
}

// =================================================================================
// Classe NecRx (Réception)
// =================================================================================

NecRx::NecRx(uint8_t pin) : _pin(pin) {}

void NecRx::begin() {
    pinMode(_pin, INPUT);
}

bool NecRx::available() {
    // Le protocole NEC commence par un BURST (LED allumée).
    // Les récepteurs IR (ex: TSOP4838) sont "Active LOW".
    // Donc au repos = HIGH. Réception signal = LOW.
    return (digitalRead(_pin) == LOW);
}

// Helper pour vérifier si une durée est dans la tolérance
static bool isToleranceOk(uint32_t measured, uint32_t expected) {
    uint32_t tolerance = (expected * NecProtocol::TOLERANCE_PERCENT) / 100;
    return (measured >= (expected - tolerance)) && (measured <= (expected + tolerance));
}

// Fonction interne pour mesurer une impulsion
// Retourne la durée en µs, ou 0 si timeout/erreur
uint32_t NecRx::measurePulse(bool state, uint32_t timeout) {
    uint32_t start = micros();
    while (digitalRead(_pin) == state) {
        if (micros() - start > timeout) {
             NEC_LOG("measurePulse TIMEOUT! State=%d, Elapsed=%lu, Timeout=%lu", state, (unsigned long)(micros() - start), timeout);
             return 0;
        }
    }
    return micros() - start;
}

NecResult NecRx::read(uint8_t& address, uint8_t& command) {
    // Attendre que la ligne passe LOW (début Header)
    // Timeout rapide si appelé sans available()
    if (digitalRead(_pin) == HIGH) return NecResult::NO_DATA;

    // 1. HEADER MARK (9ms)
    uint32_t duration = measurePulse(LOW, 12000); // Max 12ms pour 9ms
    if (duration == 0 || !isToleranceOk(duration, NecProtocol::HDR_MARK)) {
        NEC_LOG("Error: Header Mark timeout or bad duration (%lu)", duration);
        return NecResult::TIMEOUT_HEADER;
    }

    // 2. HEADER SPACE (4.5ms)
    duration = measurePulse(HIGH, 6000); // Max 6ms pour 4.5ms
    if (duration == 0 || !isToleranceOk(duration, NecProtocol::HDR_SPACE)) {
        NEC_LOG("Error: Header Space timeout or bad duration (%lu)", duration);
        return NecResult::TIMEOUT_HEADER;
    }

    NEC_LOG("Header OK. Reading bits...");

    // 3. Lecture des 32 bits
    uint32_t rawData = 0;
    for (int i = 0; i < 32; i++) {
        // BIT MARK (562µs) - Doit être LOW
        // On est large sur le timeout (1500µs) car on veut surtout mesurer le SPACE suivant
        if (measurePulse(LOW, 1500) == 0) return NecResult::TIMEOUT_DATA;

        // BIT SPACE (562µs ou 1687µs) - Doit être HIGH
        duration = measurePulse(HIGH, 3000); // Max 3ms
        if (duration == 0) return NecResult::TIMEOUT_DATA;

        if (isToleranceOk(duration, NecProtocol::ONE_SPACE)) {
            rawData |= (1UL << (31 - i));
        } else if (isToleranceOk(duration, NecProtocol::ZERO_SPACE)) {
            // C'est un 0
        } else {
            NEC_LOG("Error: Bad bit duration (%lu) at bit %d", duration, i);
            return NecResult::ERROR_PROTOCOL; // Durée invalide
        }
    }

    // 4. Bit de fin (Mark final)
    if (measurePulse(LOW, 1000) == 0) return NecResult::TIMEOUT_DATA;

    NEC_LOG("Raw Data: 0x%08lX", rawData);

    // Décodage et vérification
    uint8_t addr = (rawData >> 24) & 0xFF;
    uint8_t invAddr = (rawData >> 16) & 0xFF;
    uint8_t cmd = (rawData >> 8) & 0xFF;
    uint8_t invCmd = rawData & 0xFF;

    if ((uint8_t)(~addr) != invAddr) {
        NEC_LOG("Error: Address Checksum fail (Addr: %02X, Inv: %02X)", addr, invAddr);
        return NecResult::ERROR_PROTOCOL; // Erreur adresse
    }
    if ((uint8_t)(~cmd) != invCmd) {
        NEC_LOG("Error: Command Checksum fail (Cmd: %02X, Inv: %02X)", cmd, invCmd);
        return NecResult::ERROR_PROTOCOL;   // Erreur commande
    }

    address = addr;
    command = cmd;
    
    NEC_LOG("Frame Validated.");
    return NecResult::OK;
}

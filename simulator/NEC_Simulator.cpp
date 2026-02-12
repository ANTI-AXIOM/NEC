#include <iostream>
#include <thread>
#include <vector>
#include <atomic>

// Include du Mock
#include "Arduino.h"

// Variables globales du Mock
volatile uint8_t MOCK_PORTB = 0;
volatile uint8_t MOCK_PORTD = 0;
uint8_t pin_states[20] = {0};
std::chrono::time_point<std::chrono::steady_clock> start_time;

// Include de la librairie à tester
// Hack pour inclure le code source Arduino dans un projet C++ standard
#define NEC_DEBUG // Enable simulator logs
#include "../NEC.h"

// On va devoir inclure le .cpp directement car on ne peut pas lier facilement avec les outils standards
// Attention : il faut s'assurer que NEC.cpp n'inclut pas <Arduino.h> avec les < > qui chercheraient dans le système,
// mais "Arduino.h" ou qu'on ajoute le dossier courant au path.
// Dans NEC.h j'ai mis <Arduino.h>. Je vais devoir copier/coller ou modifier l'include path.
// Pour ce simulateur, simulons le comportement en incluant le .cpp après avoir défini l'environnement.

// Force l'inclusion de notre mock à la place du système
#define _ARDUINO_H_ // Prevent standard Arduino.h guard if it exists
#include "../NEC.cpp"

// -----------------------------------------------------------------------------
// Simulateur
// -----------------------------------------------------------------------------

// Thread de simulation physique qui connecte la sortie Tx à l'entrée Rx
// Pour simuler la transmission IR instantanée
void physicsLoop(int txPin, int rxPin, std::atomic<bool>& running) {
    while (running) {
        // Copie l'état de la pin Tx vers Rx
        // Dans la réalité, le récepteur IR inverse le signal (Active Low)
        // Tx (Carrier ON) -> IR Light -> Rx Output (LOW)
        // Tx (Carrier OFF) -> No Light -> Rx Output (HIGH)
        
        // Mais attention : NecTx génère le 38kHz.
        // NecTx::sendBurst fait osciller la pin.
        // Le récepteur TSOP contient un filtre passe-bande et un démodulateur.
        // Si la pin Tx oscille (38kHz), la sortie Rx doit être LOW.
        // Si la pin Tx est fixe (LOW), la sortie Rx doit être HIGH.
        
        // Simuler le démodulateur est complexe en temps réel pur.
        // ASTUCE : On va tricher. On va regarder si la pin Tx a changé d'état récemment.
        
        // Pour ce simulateur simple, on va supposer une connexion FILAIRE directe pour tester la logique.
        // Tx HIGH -> Rx LOW (Inversé comme un collecteur ouvert)
        // Mais NecTx génère des bursts...
        
        // APPROCHE SIMPLIFIÉE :
        // On va modifier NecTx dans le simulateur pour qu'il sorte l'enveloppe au lieu de la porteuse ?
        // Non, on veut tester le vrai code.
        
        // Donc on doit détecter l'oscillation.
        // C'est trop lourd pour un thread simple.
        
        // APPROCHE LOOPBACK LOGIQUE :
        // On va connecter Tx et Rx directement.
        // Et on va admettre que le code de lecture `NecRx` lit ce que `NecTx` écrit.
        // Problème : NecTx écrit des oscillations, NecRx attend des enveloppes (LOW/HIGH stables).
        // Le code NecRx NE MARCHERA PAS avec une connexion directe filaire car il attend des états stables (LOW pour Mark, HIGH pour Space)
        // alors que NecTx envoie des oscillations pour Mark.
        
        // Solution : Le simulateur doit implémenter un "Demodulator" virtuel.
        // Si on voit des changements rapides sur Tx, on met Rx à LOW.
        // Si Tx reste stable LOW, on met Rx à HIGH.
        static int oscillationCount = 0;
        static int lastTxState = 0;
        
        int currentTx = digitalRead(txPin); // Use digitalRead to catch port manipulation
        if (currentTx != lastTxState) {
            oscillationCount = 10; // On a vu une transition, on maintient "Carrier detected" un peu
            lastTxState = currentTx;
        }
        
        if (oscillationCount > 0) {
            pin_states[rxPin] = LOW; // Carrier detected -> Output LOW
            oscillationCount--; 
        } else {
            pin_states[rxPin] = HIGH; // No carrier -> Output HIGH
        }
        
        // Busy-wait for 10us instead of sleep for better resolution on Windows
        auto start = std::chrono::steady_clock::now();
        while(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count() < 10);
    }
}

int main() {
    start_time = std::chrono::steady_clock::now();
    std::cout << "=== NEC Protocol Simulator ===" << std::endl;

    // Configuration
    const int TX_PIN = 3;
    const int RX_PIN = 2;

    NecTx sender(TX_PIN);
    NecRx receiver(RX_PIN);

    sender.begin();
    receiver.begin();

    // Démarrage de la "physique" (Démodulateur virtuel)
    std::atomic<bool> running(true);
    std::thread physicThread(physicsLoop, TX_PIN, RX_PIN, std::ref(running));

    // Test Case 1: Envoi normal
    uint8_t addr = 0x04;
    uint8_t cmd = 0xCF;
    
    std::cout << "[SIM] Sending Address: 0x" << std::hex << (int)addr << " Command: 0x" << (int)cmd << std::endl;

    // Lancement de l'envoi dans un thread séparé pour ne pas bloquer la réception
    std::thread senderThread([&]() {
        delay(100); // Attente initiale
        sender.send(addr, cmd);
        std::cout << "[SIM] Send complete." << std::endl;
    });

    // Réception
    std::cout << "[SIM] Waiting for reception..." << std::endl;
    
    // On poll available()
    while (!receiver.available()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[SIM] Signal detected available()!" << std::endl;

    uint8_t rAddr = 0, rCmd = 0;
    NecResult result = receiver.read(rAddr, rCmd);

    if (result == NecResult::OK) {
        std::cout << "[SIM] SUCCESS! Received Address: 0x" << std::hex << (int)rAddr << " Command: 0x" << (int)rCmd << std::endl;
        if (rAddr == addr && rCmd == cmd) {
            std::cout << "[SIM] Data MATCH verified." << std::endl;
        } else {
            std::cout << "[SIM] Data MISMATCH!" << std::endl;
        }
    } else {
        std::cout << "[SIM] FAILED with error code: " << (int)result << std::endl;
    }

    senderThread.join();
    running = false;
    physicThread.join();

    return 0;
}

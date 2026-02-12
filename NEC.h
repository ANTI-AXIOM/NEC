/*************************************************************************************************************************************/
// Sujet :   Definition de la bibliotheque de fonctions NEC                                                                          //
// Auteurs :  AUGEREAU F. JOBARD L.                                                                                                  //
// Date :    14/03/2025                                                                                                              //
// Version : 1.5.3                                                                                                                   //
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

// inclusion des fichiers header des bibliothèques de fonctions Arduino
#ifndef NEC_H
#define NEC_H

#include <Arduino.h>

// Définition des codes d'erreur
enum class NecResult {
    OK,
    TIMEOUT_HEADER, // Pas d'entête détectée
    TIMEOUT_DATA,   // Timeout pendant la lecture des données
    ERROR_PROTOCOL, // Erreur de timing (hors tolérance)
    NO_DATA         // Pas de signal détecté (pour available())
};

// Constantes du protocole NEC (en microsecondes)
namespace NecProtocol {
    constexpr uint16_t HDR_MARK = 9000;
    constexpr uint16_t HDR_SPACE = 4500;
    constexpr uint16_t BIT_MARK = 562;
    constexpr uint16_t ONE_SPACE = 1687;
    constexpr uint16_t ZERO_SPACE = 562;
    // Tolérance de 20%
    constexpr uint8_t TOLERANCE_PERCENT = 20; 
}

// Debug Macros
#ifdef NEC_DEBUG
    #include <stdio.h>
    #define NEC_LOG(...) do { printf("[NEC] " __VA_ARGS__); printf("\n"); } while(0)
#else
    #define NEC_LOG(...)
#endif

// Classe pour l'émission (Optimisée AVR + Port Caching)
class NecTx {
private:
    uint8_t _pin;
    volatile uint8_t *_portReg;
    uint8_t _pinMask;

    void sendBurst(uint16_t durationMicroseconds);
    void sendSpace(uint16_t durationMicroseconds);

public:
    NecTx(uint8_t pin);
    void begin(); // Initialise le port caching
    void send(uint8_t address, uint8_t command);
};

// Classe pour la réception (Tolérante et Non-bloquante)
class NecRx {
private:
    uint8_t _pin;

    // Fonction interne pour mesurer une impulsion (niveau HAUT ou BAS)
    // Retourne 0 si timeout ou erreur, sinon la durée
    uint32_t measurePulse(bool state, uint32_t timeout);

public:
    NecRx(uint8_t pin);
    void begin();
    
    // Vérifie si une trame semble commencer (non bloquant)
    bool available(); 
    
    // Lit une trame complète (bloquant avec timeout)
    NecResult read(uint8_t& address, uint8_t& command);
};

#endif // NEC_H
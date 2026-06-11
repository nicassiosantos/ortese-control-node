// Ferramenta didatica: mostra EXATAMENTE os bytes de cada comando do protocolo
// K-Tech, sem precisar de hardware. Use para comparar com a captura do LingLong.
#include <cstdio>
#include <cstdint>
#include <vector>
#include <cmath>

void mostra_frame(const char* nome, uint8_t cmd, uint8_t id,
                  std::vector<uint8_t> dados) {
    std::vector<uint8_t> f;
    uint8_t len = dados.size();
    f.push_back(0x3E); f.push_back(cmd); f.push_back(id); f.push_back(len);
    uint8_t hdr_chk = (0x3E + cmd + id + len) & 0xFF;
    f.push_back(hdr_chk);
    if (len > 0) {
        uint8_t dchk = 0;
        for (uint8_t b : dados) { f.push_back(b); dchk += b; }
        f.push_back(dchk & 0xFF);
    }
    printf("%-28s: ", nome);
    for (uint8_t b : f) printf("%02X ", b);
    printf("\n");
    printf("%-28s  [3E=header][%02X=cmd][%02X=id][%02X=len][%02X=chk]",
           "", cmd, id, len, hdr_chk);
    if (len > 0) printf("[dados...][chk_dados]");
    printf("\n\n");
}

int main() {
    uint8_t id = 1;  // motor ID
    printf("=== PROTOCOLO DE COMUNICACAO K-TECH (motor ID=%d) ===\n\n", id);

    printf("--- Comandos SEM dados ---\n");
    mostra_frame("Motor ON (0x88)",          0x88, id, {});
    mostra_frame("Motor OFF (0x80)",         0x80, id, {});
    mostra_frame("Ler estado (0x9C)",        0x9C, id, {});
    mostra_frame("Ler multi-loop ang (0x92)",0x92, id, {});

    printf("--- Comando de TORQUE (0xA1) com exemplos ---\n");
    // converte torque -> corrente -> valor bruto (Kt=2.0, i_max=12, raw_max=2000)
    double Kt=2.0, i_max=12.0; int raw_max=2000;
    for (double torque : {1.0, 5.0, 10.0, 20.0}) {
        double current = torque / Kt;
        if (current > i_max) current = i_max;
        int16_t iq = (int16_t)(current / i_max * raw_max);
        char nome[64];
        snprintf(nome, sizeof(nome), "Torque %.0f N.m (iq=%d)", torque, iq);
        std::vector<uint8_t> dados = {
            (uint8_t)(iq & 0xFF), (uint8_t)((iq >> 8) & 0xFF)
        };
        mostra_frame(nome, 0xA1, id, dados);
    }

    printf("=== COMO CONFERIR ===\n");
    printf("1. No app LingLong, mande um comando de torque e olhe o frame TX.\n");
    printf("2. Compare com o frame que este programa gera para o mesmo torque.\n");
    printf("3. Se baterem byte a byte, o protocolo esta correto.\n");
    printf("4. O frame 3E 1F 01 00 5E que voce viu no app confirma o header+checksum.\n");
    return 0;
}
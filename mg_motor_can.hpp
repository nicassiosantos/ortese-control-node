// ============================================================================
//  mg_motor_can.hpp
//
//  Driver CAN para o motor K-Tech MG8008E-i9 (protocolo familia RMD-X/Gyems).
//  Usa SocketCAN (Linux) na Raspberry Pi.
//
//  Comandos implementados:
//    0xA1 - Torque closed-loop control: envia corrente iq alvo (proporcional
//           ao torque) e recebe na resposta: temperatura, iq atual, velocidade
//           e posicao do encoder.
//    0x9C - Read Motor Status 2: leitura de estado (usado opcionalmente).
//    0x88 - Motor ON ;  0x80 - Motor OFF.
//
//  Convencao de torque: o motor recebe iq em unidades de -2048..2047 que
//  mapeiam para a faixa de corrente do driver. A constante de torque Kt
//  (N*m/A) converte torque desejado em corrente. Ajuste Kt e i_max conforme
//  o datasheet do seu MG8008E-i9.
// ============================================================================
#ifndef CONTROL_NODE__MG_MOTOR_CAN_HPP_
#define CONTROL_NODE__MG_MOTOR_CAN_HPP_

#include <cstdint>
#include <string>

namespace control_node
{

struct MotorState
{
  double position{0.0};     // angulo do encoder [rad] (multi-volta acumulado)
  double velocity{0.0};     // velocidade [rad/s]
  double current{0.0};      // corrente iq [A]
  double torque_est{0.0};   // torque estimado = Kt * current [N*m]
  int8_t temperature{0};    // temperatura [C]
  bool   valid{false};      // resposta recebida com sucesso
};

struct MotorConfig
{
  std::string can_interface{"can0"};
  uint8_t motor_id{1};            // 1..32 ; CAN ID = 0x140 + motor_id

  // --- Valores do datasheet MG8008E-i9 V3 (Riotech) ---
  double  kt{2.0};                // constante de torque NO EIXO DE SAIDA [N*m/A]
                                  //  (rotor=0.22 x reducao 9 = 2.0)
  double  i_max{12.0};            // corrente maxima do driver [A] (Max Current)
  double  gear_ratio{9.0};        // reducao planetaria 9:1

  // Escala do comando de torque (0xA1): o protocolo envia um inteiro que
  // mapeia para a corrente. raw_max corresponde a i_max. AJUSTAR apos a
  // calibracao eletrica (ver README); 2000 e o padrao da familia RMD/K-Tech.
  int     raw_max{2000};

  // Leitura de angulo: usamos "Read Multi Loop Angle" (comando 0x92), que
  // ja retorna o angulo ACUMULADO DO EIXO DE SAIDA em 0.01 graus/LSB.
  // Assim nao dependemos de decodificar bits crus do encoder (14/16/18-bit).
  double  multiloop_to_rad{0.01 * 3.14159265358979323846 / 180.0};
                                  // 0.01 graus -> rad
};

class MGMotorCAN
{
public:
  explicit MGMotorCAN(const MotorConfig & cfg);
  ~MGMotorCAN();

  bool open();          // abre o socket CAN; retorna false em erro
  void close();

  bool motorOn();
  bool motorOff();

  // Envia comando de TORQUE (0xA1) e le a resposta com o estado do motor.
  //   torque_nm: torque desejado [N*m] (sera convertido para corrente e
  //              limitado por i_max)
  //   state    : preenchido com a resposta do motor
  // retorna true se a transacao CAN foi bem-sucedida.
  bool commandTorque(double torque_nm, MotorState & state);

  // Le o estado sem comandar torque (0x9C).
  bool readState(MotorState & state);

  // Le o angulo multi-volta do EIXO DE SAIDA (0x92). Atualiza state.position
  // com o angulo real do joelho em rad (ja considerando a reducao 9:1).
  bool readMultiLoopAngle(MotorState & state);

  bool isOpen() const { return socket_fd_ >= 0; }

private:
  bool sendFrame(const uint8_t data[8]);
  bool recvFrame(uint8_t data[8], uint32_t expected_id, int timeout_ms = 5);
  void parseStatusReply(const uint8_t data[8], MotorState & state);

  MotorConfig cfg_;
  int socket_fd_{-1};
  uint32_t tx_id_{0};   // 0x140 + id
  uint32_t rx_id_{0};   // 0x240 + id (resposta)
};

}  // namespace control_node

#endif  // CONTROL_NODE__MG_MOTOR_CAN_HPP_

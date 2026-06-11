// ============================================================================
//  mg_motor_serial.hpp
//
//  Driver RS485 (serial, via /dev/ttyUSB) para o motor K-Tech MG8008E-i9.
//  Substitui o antigo driver CAN: o hardware real usa um conversor
//  USB-RS485 (pinos A/B do motor), nao CAN.
//
//  Protocolo serial K-Tech (decodificado dos frames do LingLong Motor Tool):
//    Frame de comando: [0x3E][cmd][id][len][hdr_chk]  ([dados][data_chk])
//      - 0x3E      : header fixo
//      - cmd       : byte de comando (ex.: 0xA1 torque, 0x92 multi-loop)
//      - id        : ID do motor (1)
//      - len       : numero de bytes de dados que seguem
//      - hdr_chk   : (0x3E + cmd + id + len) & 0xFF
//      - dados     : len bytes (se houver)
//      - data_chk  : soma dos bytes de dados & 0xFF (se len>0)
//
//  A resposta tem o mesmo formato. A baudrate padrao do app e 115200.
// ============================================================================
#ifndef CONTROL_NODE__MG_MOTOR_SERIAL_HPP_
#define CONTROL_NODE__MG_MOTOR_SERIAL_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace control_node
{

struct MotorState
{
  double position{0.0};     // angulo do EIXO DE SAIDA (joelho) [rad]
  double velocity{0.0};     // velocidade do eixo de saida [rad/s]
  double current{0.0};      // corrente iq [A]
  double torque_est{0.0};   // torque estimado = Kt * current [N*m]
  int8_t temperature{0};    // temperatura [C]
  bool   valid{false};      // resposta recebida com sucesso
};

struct MotorConfig
{
  std::string port{"/dev/ttyUSB0"};   // porta serial do conversor USB-RS485
  int     baudrate{1000000};          // 1 Mbps (permite loop a 1 kHz)
  uint8_t motor_id{1};                // ID do motor

  // --- Valores do datasheet MG8008E-i9 V3 (Riotech) ---
  double  kt{2.0};                    // Torque Constant no eixo de saida [N*m/A]
  double  i_max{12.0};                // Max Current [A]
  double  gear_ratio{9.0};            // reducao 9:1

  // Escala do comando de torque: o valor iqControl vai de -2000..2000 e
  // mapeia para +-i_max. AJUSTAR raw_max apos a calibracao eletrica.
  int     raw_max{2000};

  // Multi-loop angle: 0.01 graus/LSB no protocolo -> rad
  double  multiloop_to_rad{0.01 * 3.14159265358979323846 / 180.0};

  // Modo debug: imprime cada frame TX/RX em hexadecimal no terminal.
  // Use para CONFERIR o protocolo contra a captura do app LingLong.
  bool    debug_frames{false};
};

class MGMotorSerial
{
public:
  explicit MGMotorSerial(const MotorConfig & cfg);
  ~MGMotorSerial();

  bool open();          // abre a porta serial; false em erro
  void close();
  bool isOpen() const { return fd_ >= 0; }

  bool motorOn();
  bool motorOff();

  // Comando de torque (0xA1) + leitura do estado na resposta.
  bool commandTorque(double torque_nm, MotorState & state);

  // Leitura de estado sem comandar (0x9C).
  bool readState(MotorState & state);

  // Angulo multi-volta do eixo de saida (0x92) -> state.position [rad].
  bool readMultiLoopAngle(MotorState & state);

private:
  // monta e envia um frame [3E][cmd][id][len][chk] (+dados+chk)
  bool sendCommand(uint8_t cmd, const std::vector<uint8_t> & data = {});
  // le uma resposta; preenche 'payload' com os bytes de dados
  bool readReply(uint8_t expected_cmd, std::vector<uint8_t> & payload,
                 int timeout_ms = 20);
  void parseStatusReply(const std::vector<uint8_t> & p, MotorState & state);

  MotorConfig cfg_;
  int fd_{-1};
};

}  // namespace control_node

#endif  // CONTROL_NODE__MG_MOTOR_SERIAL_HPP_
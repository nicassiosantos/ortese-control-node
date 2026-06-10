// ============================================================================
//  mg_motor_can.cpp  -  implementacao do driver CAN (SocketCAN)
// ============================================================================
#include "control_node/mg_motor_can.hpp"

#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>

namespace control_node
{

MGMotorCAN::MGMotorCAN(const MotorConfig & cfg)
: cfg_(cfg)
{
  tx_id_ = 0x140 + cfg_.motor_id;   // comando -> motor
  rx_id_ = 0x240 + cfg_.motor_id;   // resposta <- motor
}

MGMotorCAN::~MGMotorCAN()
{
  close();
}

bool MGMotorCAN::open()
{
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "[MGMotorCAN] erro ao criar socket CAN\n";
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, cfg_.can_interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "[MGMotorCAN] interface " << cfg_.can_interface
              << " nao encontrada\n";
    close();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr),
             sizeof(addr)) < 0) {
    std::cerr << "[MGMotorCAN] erro no bind do socket CAN\n";
    close();
    return false;
  }

  // filtro: so recebe a resposta deste motor
  struct can_filter filt;
  filt.can_id = rx_id_;
  filt.can_mask = CAN_SFF_MASK;
  ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt));

  return true;
}

void MGMotorCAN::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool MGMotorCAN::sendFrame(const uint8_t data[8])
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = tx_id_;
  frame.can_dlc = 8;
  std::memcpy(frame.data, data, 8);
  ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

bool MGMotorCAN::recvFrame(uint8_t data[8], uint32_t expected_id, int timeout_ms)
{
  // espera simples por leitura com timeout via select
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(socket_fd_, &rfds);
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeout_ms * 1000;
  int r = ::select(socket_fd_ + 1, &rfds, nullptr, nullptr, &tv);
  if (r <= 0) return false;   // timeout ou erro

  struct can_frame frame;
  ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
  if (n < static_cast<ssize_t>(sizeof(frame))) return false;
  if (frame.can_id != expected_id) return false;
  std::memcpy(data, frame.data, 8);
  return true;
}

bool MGMotorCAN::motorOn()
{
  uint8_t tx[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
  if (!sendFrame(tx)) return false;
  uint8_t rx[8];
  return recvFrame(rx, rx_id_, 10);
}

bool MGMotorCAN::motorOff()
{
  uint8_t tx[8] = {0x80, 0, 0, 0, 0, 0, 0, 0};
  if (!sendFrame(tx)) return false;
  uint8_t rx[8];
  return recvFrame(rx, rx_id_, 10);
}

bool MGMotorCAN::commandTorque(double torque_nm, MotorState & state)
{
  // torque_nm e o torque desejado NO EIXO DE SAIDA (no joelho) [N*m].
  // Kt (cfg_.kt = 2.0) ja e a constante de torque do eixo de saida
  // (rotor 0.22 x reducao 9). Logo: corrente = torque / Kt, direto.
  double current = torque_nm / cfg_.kt;

  // limita pela corrente maxima do driver (12 A no MG8008E-i9)
  if (current >  cfg_.i_max) current =  cfg_.i_max;
  if (current < -cfg_.i_max) current = -cfg_.i_max;

  // converte corrente [A] -> valor bruto do comando 0xA1
  // (raw_max corresponde a i_max; AJUSTAR raw_max apos calibracao eletrica)
  int16_t iq_raw = static_cast<int16_t>(
      current / cfg_.i_max * static_cast<double>(cfg_.raw_max));

  // monta o frame do comando 0xA1 (Torque closed-loop control)
  uint8_t tx[8] = {0};
  tx[0] = 0xA1;
  // bytes 1..3 reservados (0)
  tx[4] = static_cast<uint8_t>(iq_raw & 0xFF);          // iq baixo
  tx[5] = static_cast<uint8_t>((iq_raw >> 8) & 0xFF);   // iq alto
  // bytes 6..7 reservados (0)

  if (!sendFrame(tx)) { state.valid = false; return false; }

  uint8_t rx[8];
  if (!recvFrame(rx, rx_id_, 5)) { state.valid = false; return false; }

  parseStatusReply(rx, state);

  // O angulo na resposta de 0xA1 e single-loop (1 volta do rotor). Para o
  // controle usamos o multi-loop do eixo de saida; o control_node chama
  // readMultiLoopAngle separadamente quando precisa do angulo do joelho.
  return state.valid;
}

bool MGMotorCAN::readState(MotorState & state)
{
  uint8_t tx[8] = {0x9C, 0, 0, 0, 0, 0, 0, 0};
  if (!sendFrame(tx)) { state.valid = false; return false; }
  uint8_t rx[8];
  if (!recvFrame(rx, rx_id_, 5)) { state.valid = false; return false; }
  parseStatusReply(rx, state);
  return state.valid;
}

void MGMotorCAN::parseStatusReply(const uint8_t data[8], MotorState & state)
{
  // Layout da resposta (0xA1 / 0x9C) no protocolo K-Tech/RMD-X V3.x:
  //  data[0] = comando (eco)
  //  data[1] = temperatura [C]
  //  data[2..3] = iq atual (int16, LSB primeiro) -- corrente do rotor
  //  data[4..5] = velocidade [graus/s] (int16, LSB primeiro) -- do rotor
  //  data[6..7] = posicao single-loop do encoder (uint16, LSB primeiro)
  state.temperature = static_cast<int8_t>(data[1]);

  int16_t iq = static_cast<int16_t>(data[2] | (data[3] << 8));
  state.current = static_cast<double>(iq) / static_cast<double>(cfg_.raw_max)
                  * cfg_.i_max;
  // torque no EIXO DE SAIDA = corrente * Kt (Kt ja e do eixo de saida)
  state.torque_est = state.current * cfg_.kt;

  // velocidade do rotor [graus/s] -> eixo de saida [rad/s] (divide pela reducao)
  int16_t speed_dps = static_cast<int16_t>(data[4] | (data[5] << 8));
  state.velocity = static_cast<double>(speed_dps) * M_PI / 180.0
                   / cfg_.gear_ratio;

  // OBS: a posicao single-loop (data[6..7]) NAO e usada para o controle.
  // O angulo do joelho vem de readMultiLoopAngle(). Mantemos state.position
  // inalterado aqui para nao sobrescrever o multi-loop.

  state.valid = true;
}

bool MGMotorCAN::readMultiLoopAngle(MotorState & state)
{
  // Comando 0x92: Read Multi Loop Angle. Retorna o angulo acumulado do
  // EIXO DE SAIDA como int64 (8 bytes) em unidade de 0.01 graus/LSB.
  uint8_t tx[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
  if (!sendFrame(tx)) { state.valid = false; return false; }
  uint8_t rx[8];
  if (!recvFrame(rx, rx_id_, 5)) { state.valid = false; return false; }

  // monta o int64 a partir dos 8 bytes (LSB primeiro)
  int64_t angle_raw = 0;
  for (int i = 0; i < 8; ++i) {
    angle_raw |= static_cast<int64_t>(rx[i]) << (8 * i);
  }
  // o byte 0 e o eco do comando em alguns firmwares; se for o caso, o
  // angulo vem em rx[1..7]. Tratamos o formato padrao (8 bytes de dado).
  // 0.01 graus/LSB -> rad
  state.position = static_cast<double>(angle_raw) * cfg_.multiloop_to_rad;
  state.valid = true;
  return true;
}

}  // namespace control_node

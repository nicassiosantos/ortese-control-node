// ============================================================================
//  mg_motor_serial.cpp  -  driver RS485 (serial) para o motor K-Tech
// ============================================================================
#include "control_node/mg_motor_serial.hpp"

#include <cstring>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <iostream>

namespace control_node
{

MGMotorSerial::MGMotorSerial(const MotorConfig & cfg)
: cfg_(cfg) {}

MGMotorSerial::~MGMotorSerial() { close(); }

static speed_t baudToSpeed(int baud)
{
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 1000000: return B1000000;
    default:      return B115200;
  }
}

bool MGMotorSerial::open()
{
  fd_ = ::open(cfg_.port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) {
    std::cerr << "[MGMotorSerial] nao foi possivel abrir " << cfg_.port << "\n";
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[MGMotorSerial] tcgetattr falhou\n";
    close();
    return false;
  }

  speed_t spd = baudToSpeed(cfg_.baudrate);
  cfsetospeed(&tty, spd);
  cfsetispeed(&tty, spd);

  // 8N1, sem controle de fluxo, modo raw
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);   // sem paridade
  tty.c_cflag &= ~CSTOPB;              // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;             // sem hardware flow control
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;   // leitura nao-bloqueante; usamos select()

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "[MGMotorSerial] tcsetattr falhou\n";
    close();
    return false;
  }

  // limpa qualquer lixo no buffer
  tcflush(fd_, TCIOFLUSH);
  // volta ao modo bloqueante controlado por select
  fcntl(fd_, F_SETFL, 0);
  return true;
}

void MGMotorSerial::close()
{
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool MGMotorSerial::sendCommand(uint8_t cmd, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> frame;
  uint8_t len = static_cast<uint8_t>(data.size());

  // cabecalho: 3E cmd id len
  frame.push_back(0x3E);
  frame.push_back(cmd);
  frame.push_back(cfg_.motor_id);
  frame.push_back(len);
  // checksum do cabecalho
  uint8_t hdr_chk = static_cast<uint8_t>(0x3E + cmd + cfg_.motor_id + len);
  frame.push_back(hdr_chk);

  // dados + checksum dos dados (se houver)
  if (len > 0) {
    uint8_t data_chk = 0;
    for (uint8_t b : data) { frame.push_back(b); data_chk += b; }
    frame.push_back(data_chk);
  }

  ssize_t n = ::write(fd_, frame.data(), frame.size());

  if (cfg_.debug_frames) {
    std::cerr << "[TX] ";
    for (uint8_t b : frame) {
      char buf[4]; std::snprintf(buf, sizeof(buf), "%02X ", b);
      std::cerr << buf;
    }
    std::cerr << "\n";
  }

  return n == static_cast<ssize_t>(frame.size());
}

bool MGMotorSerial::readReply(uint8_t expected_cmd,
                              std::vector<uint8_t> & payload, int timeout_ms)
{
  payload.clear();

  // le ate encontrar o header 0x3E e montar um frame completo
  // estrutura: 3E cmd id len hdr_chk [dados(len) data_chk]
  uint8_t hdr[5];
  int got = 0;

  auto readByte = [&](uint8_t & b) -> bool {
    fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = timeout_ms * 1000;
    if (::select(fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) return false;
    return ::read(fd_, &b, 1) == 1;
  };

  // procura o header 0x3E
  uint8_t b;
  bool found = false;
  for (int i = 0; i < 64; ++i) {
    if (!readByte(b)) return false;
    if (b == 0x3E) { found = true; break; }
  }
  if (!found) return false;

  hdr[0] = 0x3E;
  for (got = 1; got < 5; ++got) {
    if (!readByte(hdr[got])) return false;
  }

  uint8_t cmd = hdr[1];
  uint8_t len = hdr[3];
  uint8_t hdr_chk = hdr[4];
  if (static_cast<uint8_t>(hdr[0] + hdr[1] + hdr[2] + hdr[3]) != hdr_chk) {
    return false;  // checksum de cabecalho invalido
  }
  if (cmd != expected_cmd) {
    // resposta de outro comando; ignora
    if (cfg_.debug_frames) {
      std::cerr << "[RX?] comando inesperado: esperava "
                << std::hex << static_cast<int>(expected_cmd)
                << " recebeu " << static_cast<int>(cmd) << std::dec << "\n";
    }
    return false;
  }

  if (len > 0) {
    payload.resize(len);
    for (int i = 0; i < len; ++i) {
      if (!readByte(payload[i])) return false;
    }
    uint8_t data_chk;
    if (!readByte(data_chk)) return false;
    uint8_t sum = 0;
    for (uint8_t x : payload) sum += x;
    if (sum != data_chk) return false;  // checksum de dados invalido
  }

  if (cfg_.debug_frames) {
    std::cerr << "[RX] 3E ";
    for (int i = 1; i < 5; ++i) {
      char buf[4]; std::snprintf(buf, sizeof(buf), "%02X ", hdr[i]);
      std::cerr << buf;
    }
    for (uint8_t b : payload) {
      char buf[4]; std::snprintf(buf, sizeof(buf), "%02X ", b);
      std::cerr << buf;
    }
    std::cerr << "\n";
  }

  return true;
}

bool MGMotorSerial::motorOn()
{
  // 0x88 = Motor ON
  if (!sendCommand(0x88)) return false;
  std::vector<uint8_t> p;
  return readReply(0x88, p, 100);  // timeout generoso p/ Motor ON
}

bool MGMotorSerial::motorOff()
{
  // 0x80 = Motor OFF
  if (!sendCommand(0x80)) return false;
  std::vector<uint8_t> p;
  return readReply(0x80, p, 100);  // timeout generoso p/ Motor OFF
}

bool MGMotorSerial::commandTorque(double valor, MotorState & state)
{
  // 'valor' e interpretado conforme cfg_.command_mode:
  //   "raw"    -> valor e o iq BRUTO direto (igual ao app LingLong)
  //   "torque" -> valor e o torque em N.m, convertido para iq bruto
  int16_t iq;

  if (cfg_.command_mode == "raw") {
    // valor bruto direto; satura no intervalo do protocolo
    double v = valor;
    if (v >  cfg_.raw_max) v =  cfg_.raw_max;
    if (v < -cfg_.raw_max) v = -cfg_.raw_max;
    iq = static_cast<int16_t>(v);
  } else {
    // modo "torque": converte N.m -> corrente -> iq bruto
    double current = valor / cfg_.kt;                 // N.m -> A (eixo saida)
    if (current >  cfg_.i_max) current =  cfg_.i_max;
    if (current < -cfg_.i_max) current = -cfg_.i_max;
    iq = static_cast<int16_t>(
        current / cfg_.i_max * static_cast<double>(cfg_.raw_max));
  }

  // 0xA1 = torque closed-loop; 2 bytes de dados (iq, LSB primeiro)
  std::vector<uint8_t> data = {
    static_cast<uint8_t>(iq & 0xFF),
    static_cast<uint8_t>((iq >> 8) & 0xFF)
  };
  if (!sendCommand(0xA1, data)) { state.valid = false; return false; }

  std::vector<uint8_t> p;
  if (!readReply(0xA1, p, 20)) { state.valid = false; return false; }
  parseStatusReply(p, state);
  return state.valid;
}

bool MGMotorSerial::readState(MotorState & state)
{
  if (!sendCommand(0x9C)) { state.valid = false; return false; }
  std::vector<uint8_t> p;
  if (!readReply(0x9C, p, 20)) { state.valid = false; return false; }
  parseStatusReply(p, state);
  return state.valid;
}

bool MGMotorSerial::readMultiLoopAngle(MotorState & state)
{
  // 0x92 = read multi loop angle; resposta traz int64 (0.01 graus/LSB)
  if (!sendCommand(0x92)) { state.valid = false; return false; }
  std::vector<uint8_t> p;
  if (!readReply(0x92, p, 20)) { state.valid = false; return false; }
  if (p.size() < 8) { state.valid = false; return false; }

  int64_t raw = 0;
  for (int i = 0; i < 8; ++i) {
    raw |= static_cast<int64_t>(p[i]) << (8 * i);
  }
  state.position = static_cast<double>(raw) * cfg_.multiloop_to_rad;
  state.valid = true;
  return true;
}

void MGMotorSerial::parseStatusReply(const std::vector<uint8_t> & p,
                                     MotorState & state)
{
  // Layout CONFIRMADO pela captura do LingLong (resposta 0xA1, len=07):
  //   resposta: 3E A1 01 07 E7 [19 F5 FF 00 00 26 EF] 22
  //   p[0]    = temperatura [C]              (0x19 = 25 C, conferido na tela)
  //   p[1..2] = torque current iq (int16 LE) (0xFFF5 = -11, conferido)
  //   p[3..4] = velocidade [graus/s] (int16) (0x0000 = 0, conferido)
  //   p[5..6] = encoder posicao (uint16 LE)  (0xEF26 = 61222, conferido)
  if (p.size() < 7) { state.valid = false; return; }

  state.temperature = static_cast<int8_t>(p[0]);

  int16_t iq = static_cast<int16_t>(p[1] | (p[2] << 8));
  // a corrente reportada e o valor bruto iq; converte para Amperes
  state.current = static_cast<double>(iq) / static_cast<double>(cfg_.raw_max)
                  * cfg_.i_max;
  state.torque_est = state.current * cfg_.kt;   // torque no eixo de saida

  int16_t speed_dps = static_cast<int16_t>(p[3] | (p[4] << 8));
  state.velocity = static_cast<double>(speed_dps) * M_PI / 180.0
                   / cfg_.gear_ratio;           // rotor -> eixo de saida

  // p[5..6] = encoder single-loop (0..65535). Nao usamos para o controle
  // (o angulo do joelho vem de readMultiLoopAngle / 0x92), mas guardamos
  // para diagnostico em state.current/etc. ja preenchidos acima.

  state.valid = true;
}

}  // namespace control_node
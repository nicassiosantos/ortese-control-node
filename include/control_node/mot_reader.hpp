// ============================================================================
//  mot_reader.hpp
//
//  Leitor de arquivos .mot / .sto do OpenSim. Extrai uma coluna nomeada
//  (ex.: knee_angle_r) e o vetor de tempo, detectando se os angulos estao
//  em graus (cabecalho inDegrees=yes) para converter em radianos.
//
//  Formato .mot:
//    <nome>
//    nRows=...        nColumns=...
//    inDegrees=yes/no
//    ...
//    endheader
//    time  col1  col2  ...   (linha de nomes)
//    <dados separados por espaco/tab>
// ============================================================================
#ifndef CONTROL_NODE__MOT_READER_HPP_
#define CONTROL_NODE__MOT_READER_HPP_

#include <string>
#include <vector>

namespace control_node
{

struct MotData
{
  std::vector<double> time;     // vetor de tempo [s]
  std::vector<double> values;   // coluna pedida, em RADIANOS
  bool ok{false};
  std::string error;
};

class MotReader
{
public:
  // Le o arquivo path e extrai a coluna column_name (alem de 'time').
  // Converte para radianos se o cabecalho indicar inDegrees=yes.
  static MotData read(const std::string & path, const std::string & column_name);
};

}  // namespace control_node

#endif  // CONTROL_NODE__MOT_READER_HPP_

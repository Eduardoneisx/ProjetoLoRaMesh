[README.md](https://github.com/user-attachments/files/34336989/README.md)
# ProjetoLoRaMesh

Firmware de uma rede mesh LoRa para monitoramento de temperatura e umidade em ambientes com obstrução severa, desenvolvida no LabTermo (UFSC) em um projeto de iniciação científica orientado pelo Prof. Saulo Güths.

O projeto nasceu da necessidade de monitorar um museu com paredes espessas, onde a topologia em estrela do LoRaWAN deixa áreas sem cobertura e uma malha Wi-Fi exigiria nós demais. A solução é um protocolo multi-hop leve: os próprios nós sensores repassam as mensagens dos vizinhos até a base, sem ACK, sem tabela de rotas e com o mínimo possível de tráfego de controle. A base agrega os dados e envia para uma planilha Google, que alimenta os dashboards de acompanhamento.

## Resultados

Os ensaios foram feitos no campus da UFSC, de propósito em condição desfavorável: base dentro de um prédio, sinal atravessando várias paredes, vegetação densa e uma segunda edificação, com ruído ambiente medido em -87 dBm no canal.

**Alcance.** Entre 150 e 200 m no mesmo percurso, dependendo do dia. Em dia chuvoso o enlace chegou a 150 m e em dia seco a 193 m, o que aponta a umidade da vegetação como fator determinante. O enlace se manteve com RSSI de até -93 dBm, abaixo do piso de ruído, graças ao ganho de processamento do air rate baixo.

**Taxa de entrega em 1 hora.** 92,7% no nó de um salto e 51,1% no nó de dois saltos, numa topologia sem rota alternativa.

**Operação contínua por 3 horas** com quatro nós sensores, cerca de 180 mensagens esperadas por nó:

| Nó | Rota | Sem rota redundante | Com rota redundante |
|---|---|---|---|
| 1 | direta | 90,5% | 89,4% |
| 2 | direta | 99,9% | 99,4% |
| 3 | 2 saltos | 52,2% | 58,3% |
| 4 | direta | 98,9% | 91,1% |

Quando um segundo vizinho passou a alcançar o nó 3, as perdas dele caíram cerca de 12,8% sem nenhuma configuração extra, porque a própria regra de encaminhamento aproveitou o caminho novo. A taxa agregada ficou em 85,4% e 84,6% nas duas configurações.

Para grandezas de variação lenta como temperatura e umidade, uma leitura perdida é coberta pela seguinte cerca de um minuto depois, então esse nível de entrega atende a aplicação. O próximo passo é a implantação no museu e testes com topologias mais densas e mais saltos.

## Hardware

- ESP32 (esp32dev)
- Módulo LoRa EBYTE E220-900T30D em 915 MHz, com antena de +3 dBi, ligado na `Serial2` a 9600 baud
- Sensor DHT22 nos nós sensores
- PCB própria, com M0/M1 fixados em hardware no modo de operação normal e desacoplamento dimensionado para os picos de cerca de 620 mA durante a transmissão

| Sinal | GPIO |
|---|---|
| E220 TX → ESP32 RX | 16 |
| E220 RX ← ESP32 TX | 17 |
| E220 AUX | 19 |
| DHT22 (dados) | 25 |

O E220 opera com potência de 30 dBm, air rate de 2,4 kbps, byte de RSSI anexado a cada pacote recebido e listen before talk habilitado. Registradores usados:

| Registrador | Valor |
|---|---|
| REG0 | `0x62` |
| REG1 | `0x20` |
| REG2 | `0x32` |
| REG3 | `0x90` |

## Como a rede funciona

### Formato do pacote

Cada mensagem tem 12 bytes, empacotados sem padding:

| Campo | Bytes | Função |
|---|---|---|
| `startByte` | 1 | Sempre `0xAA`, usado para ressincronizar o stream da UART |
| `msgId` | 1 | Identificador da mensagem no nó de origem |
| `src` | 1 | ID do nó de origem |
| `ttl` | 1 | Saltos restantes (começa em 3) |
| `srcHopsToBase` | 1 | Distância até a base de quem fez a última transmissão |
| `type` | 1 | `0` = dado, `1` = beacon |
| `payload` | 5 | Temperatura e umidade (bytes 0 e 1), restante reservado |
| `checkSum` | 1 | CRC-8 (polinômio `0x07`) |

Na recepção chega um 13º byte, o RSSI que o E220 adiciona ao final.

### Descoberta de rota

O roteamento segue um gradiente de distância até a base. Só a base emite beacons, a cada 90 s com jitter, anunciando distância zero. Os nós que não alcançam a base aprendem a distância de forma passiva, olhando o campo `srcHopsToBase` dos pacotes de dados que passam por eles. A atualização só acontece com RSSI acima de -105 dBm, para não aprender rota por enlace ruim. Se a rota não for confirmada por três intervalos de beacon, ela expira e o nó para de enviar até a topologia convergir de novo.

### Repasse

Quando um nó recebe um pacote de dados, ele decide se repassa:

- se quem transmitiu está mais longe da base do que ele, repassa sempre;
- se está na mesma distância, repassa com uma probabilidade que depende do RSSI (15% com sinal forte, 30% com sinal fraco). Sinal forte indica que outros vizinhos provavelmente também ouviram; sinal fraco sugere que este nó pode ser o único elo confiável, o que ajuda a contornar enlaces assimétricos;
- se está mais perto da base, não repassa.

Pacotes a repassar entram numa fila com atraso aleatório de 50 a 300 ms, para que vizinhos que ouviram a mesma mensagem não retransmitam juntos. Duplicatas são descartadas por um cache de hash indexado pelo par (origem, msgId), com 64 posições, sondagem linear e expiração de 15 s por entrada. Cada posição pode estar livre, ocupada ou liberada; a posição liberada funciona como lápide, que pode ser reaproveitada mas não interrompe a busca por chaves mais adiante na cadeia. A expiração é tratada durante a própria consulta, sem tarefa periódica, e a tabela é reconstruída só com as entradas válidas quando mais de três quartos das posições já foram usadas. Tudo fica em memória estática.

### Decisões de projeto

O air rate baixo foi escolhido porque, sob obstrução, ele decide se o enlace existe ou não. O custo é um tempo no ar maior por mensagem, e quase todo o resto do protocolo vem daí: o pacote foi enxugado até 12 bytes (CRC de 8 bits, sem campos de tamanho ou contagem de saltos), os beacons ficaram restritos à base e não há ACK, que dobraria o tráfego num canal já escasso. Para evitar colisões que se repetem quando dois nós entram em fase, toda a temporização tem jitter de ±33%, e o LBT do módulo cuida da concorrência instantânea.

### Tarefas FreeRTOS

Em todos os nós rodam duas tarefas no core 1:

- **taskRX** sincroniza no byte de início, lê o pacote e o RSSI, valida o CRC, descarta duplicatas e despacha para o tratador de dado ou de beacon;
- **taskPeriodic** envia os dados do sensor (nós, a cada 60 s) ou os beacons (base), verifica a expiração de rota e processa a fila de repasse.

O estado de rota e a fila de repasse ficam encapsulados nas classes `RouteState` e `FwdQueue`, protegidas por mutex. Antes de transmitir, o firmware espera o pino AUX do E220 indicar que o módulo está livre.

### Base e envio para a planilha

A base acumula as leituras por nó e roda duas tarefas no core 0, junto com o Wi-Fi:

- a cada **5 min** envia o valor mais recente de cada nó, marcando como offline quem não reportou nesse intervalo;
- a cada **60 min** envia a média horária de cada nó com o número de amostras e zera os acumuladores.

Os envios são POSTs em JSON para um Google Apps Script, autenticados por token. Na planilha, os dados alimentam dashboards no Looker Studio e no Power BI.

## Estrutura do repositório

```
ProjetoLoRaMesh/
├── platformio.ini                  # ambientes de compilação (base e nós)
└── src/
    ├── LoRaMeshFirmwareOficial.cpp # protocolo mesh, tarefas e sensores
    ├── envio_sheets.cpp/.h         # agregação e envio da base para o Google Sheets
    └── secrets.h                   # credenciais locais (não versionado)
```

## Como compilar

O projeto usa PlatformIO, e o papel de cada placa é definido pelas flags `MEU_ID` e `BASE_ID` nos ambientes `base` e `node1` a `node4` do `platformio.ini`; para mais nós, basta criar um ambiente com outro `MEU_ID`. Antes de gravar a base, crie `src/secrets.h`:

```cpp
#pragma once
#define WIFI_SSID_SECRET   "sua-rede"
#define WIFI_PASS_SECRET   "sua-senha"
#define SCRIPT_URL_SECRET  "https://script.google.com/macros/s/.../exec"
#define TOKEN_SECRET       "seu-token"
```

Depois é só gravar o ambiente desejado, por exemplo `pio run -e node1 -t upload`.

## Autor

Eduardo Gonçalves Neis, graduando em Engenharia de Controle e Automação na UFSC e bolsista de iniciação científica no LabTermo.

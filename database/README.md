# Banco de dados da rede LoRa mesh

Camada de armazenamento e análise da rede de monitoramento ambiental do museu. A base ESP32 recebe os pacotes dos nós via LoRa e imprime cada leitura na serial; um script em Python lê essas linhas e grava no PostgreSQL, onde views prontas respondem às perguntas de operação da rede e de conservação do acervo.

```
nós sensores → LoRa mesh → base ESP32 → USB/serial → Python → PostgreSQL → views / Power BI
```

## Estrutura

| Arquivo | Função |
|---|---|
| `schema.sql` | Cria as tabelas `nos` e `leituras`, o índice e as views |
| `ingestao_serial.py` | Lê a serial da base, interpreta as linhas e insere no banco |
| `config.example.py` | Modelo da string de conexão (copie para `config.py`) |

## Formato esperado na serial

```
Dado: src=2 TTL=3 rssi=-35 temp=24 umid=82
```

O parser procura `Dado:` em qualquer posição da linha e lê os campos no formato `chave=valor`, então a ordem dos campos não importa. Linhas de debug da base (beacons, status HTTP) são ignoradas. O número de saltos é calculado como `TTL_INICIAL - TTL + 1`: o TTL começa em 3 e só é decrementado nos repasses, então um nó que fala direto com a base chega com TTL 3 e conta 1 salto.

## Views

| View | Pergunta que responde |
|---|---|
| `v_atual` | Qual a leitura mais recente de cada nó? |
| `v_entrega_horaria` | Quantas mensagens cada nó entregou por hora, em relação às 60 esperadas? |
| `v_lacunas` | Quando um nó ficou mais de 3 minutos sem comunicar? |
| `v_media_diaria` | Qual a temperatura e umidade média por local e por dia? |
| `v_variacao_ur` | Em que dias a umidade de um local oscilou mais de 5 pontos? |

A última é a mais relevante para conservação preventiva: oscilações bruscas de umidade relativa causam mais dano ao acervo do que um valor alto e estável.

## Como rodar

1. Instale o PostgreSQL e crie o banco: `CREATE DATABASE museu;`
2. Rode `schema.sql` no banco `museu` e cadastre os nós reais na tabela `nos`.
3. Copie `config.example.py` para `config.py` e coloque sua senha.
4. Instale as dependências: `pip install -r requirements.txt`
5. Ajuste `PORTA` em `ingestao_serial.py`, feche o monitor serial da IDE e rode:
   `python ingestao_serial.py`

Atenção: `schema.sql` começa com `DROP TABLE ... CASCADE` e apaga os dados existentes. Use apenas para criar o banco do zero.

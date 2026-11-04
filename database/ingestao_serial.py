import serial
import psycopg
from config import DB_URL

PORTA = "COM4"          # porta serial da base (Gerenciador de Dispositivos)
BAUD = 115200
TTL_INICIAL = 3   # TTL com que o nó de origem envia o pacote (ver firmware)

conn = psycopg.connect(DB_URL, autocommit=True)

def interpretar(linha):
    # Formato: Dado: src=2 TTL=3 rssi=-35 temp=24 umid=82
    pos = linha.find("Dado:")
    if pos == -1:
        return None
    campos = {}
    for parte in linha[pos + 5:].split():
        if "=" in parte:
            chave, valor = parte.split("=", 1)
            campos[chave.lower()] = valor
    try:
        return (
            int(campos["src"]),
            float(campos["temp"]),
            float(campos["umid"]),
            int(campos["rssi"]),
            TTL_INICIAL - int(campos["ttl"]) + 1,   # enlaces de rádio até a base
        )
    except (KeyError, ValueError):
        return None

with serial.Serial(PORTA, BAUD, timeout=1) as ser:
    print("Lendo", PORTA)
    while True:
        linha = ser.readline().decode(errors="ignore").strip()
        if not linha:
            continue
        dados = interpretar(linha)
        if dados is None:
            print("ignorada:", linha)
            continue
        try:
            conn.execute(
                """INSERT INTO leituras (no_id, recebido_em, temperatura, umidade, rssi, saltos)
                   VALUES (%s, NOW(), %s, %s, %s, %s)""",
                dados
            )
            print("gravada:", dados)
        except psycopg.Error as e:
            print("erro no banco:", e)
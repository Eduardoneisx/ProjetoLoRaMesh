DROP TABLE IF EXISTS leituras, nos CASCADE;

CREATE TABLE nos (
    id           SMALLINT PRIMARY KEY,
    nome         VARCHAR(40),
    local        VARCHAR(80),
    instalado_em TIMESTAMP
);

CREATE TABLE leituras (
    id          BIGSERIAL PRIMARY KEY,
    no_id       SMALLINT REFERENCES nos(id),
    recebido_em TIMESTAMP NOT NULL,
    temperatura NUMERIC(5,2),
    umidade     NUMERIC(5,2),
    rssi        SMALLINT,
    saltos      SMALLINT
);

CREATE INDEX ON leituras (no_id, recebido_em);

-- Leitura mais recente de cada nó
CREATE VIEW v_atual AS
SELECT DISTINCT ON (l.no_id)
       n.local, l.no_id, l.recebido_em, l.temperatura, l.umidade, l.rssi, l.saltos
FROM leituras l
JOIN nos n ON n.id = l.no_id
ORDER BY l.no_id, l.recebido_em DESC;

-- Taxa de entrega por nó e por hora
CREATE VIEW v_entrega_horaria AS
SELECT no_id,
       date_trunc('hour', recebido_em) AS hora,
       COUNT(*) AS recebidas,
       ROUND(COUNT(*) / 60.0 * 100, 1) AS entrega_pct
FROM leituras
GROUP BY no_id, hora;

-- Lacunas de comunicação (intervalo entre mensagens maior que 3 min)
CREATE VIEW v_lacunas AS
SELECT no_id, recebido_em, intervalo
FROM (
    SELECT no_id, recebido_em,
           recebido_em - LAG(recebido_em)
               OVER (PARTITION BY no_id ORDER BY recebido_em) AS intervalo
    FROM leituras
) t
WHERE intervalo > INTERVAL '3 minutes';

-- Média diária por local
CREATE VIEW v_media_diaria AS
SELECT n.local,
       date_trunc('day', l.recebido_em) AS dia,
       ROUND(AVG(l.temperatura), 1) AS temp_media,
       ROUND(AVG(l.umidade), 1)     AS umid_media
FROM leituras l
JOIN nos n ON n.id = l.no_id
GROUP BY n.local, dia;

-- Dias com variação de umidade acima de 5 pontos por local
CREATE VIEW v_variacao_ur AS
SELECT n.local,
       date_trunc('day', l.recebido_em) AS dia,
       MAX(l.umidade) - MIN(l.umidade) AS variacao_ur
FROM leituras l
JOIN nos n ON n.id = l.no_id
GROUP BY n.local, dia
HAVING MAX(l.umidade) - MIN(l.umidade) > 5;

-- Nós da rede (ajuste para os IDs e locais reais)
INSERT INTO nos VALUES
  (1, 'Nó 1', 'Reserva técnica', NOW()),
  (2, 'Nó 2', 'Sala de exposição', NOW());
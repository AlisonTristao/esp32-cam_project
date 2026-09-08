# Compressão adaptativa da câmera

A captura aplica crop, reamostragem por `size` e quantização por `depth`.
Em seguida executa filtro 3×3, agrupamento por tolerância e seleção RAW/RLE.
A tentativa de compressão está sempre ativa, para todas as profundidades.

## Configuração

Exemplo para 60×60 em 4 bits, partindo da configuração padrão:

```text
cam set_size 60
cam set_depth 4
cam set_delta 2
cam delta
cam status
cam save
```

Se a configuração atual de size/depth impedir uma transição, reduza primeiro
`size`. O limite continua calculado pelo pior caso RAW: o corpo completo deve
caber em 1.900 bytes, incluindo o cabeçalho de quatro bytes.

`delta` aceita 0–255 e é medido nos níveis já quantizados (0–15 em 4 bits).
A tolerância efetiva é limitada ao máximo do depth; em 1 bit é sempre zero.
`cam delta` mostra o valor configurado e o efetivo. `delta=0` agrupa apenas
valores iguais, mas o filtro de ruído continua ativo e pode alterar pixels.

`cam status` mostra RAW/encoded bytes do último frame processado, incluindo
o cabeçalho da matriz, e o formato escolhido. Esses valores são atualizados
quando há assinatura ativa de `camera.matrix`; não incluem BTP, criptografia
ou cabeçalhos de rádio e não comprovam entrega do frame.

`cam save`/`cam load` persistem delta com threshold/size/depth/crop no SD.
`camera.cfg` permanece com 28 bytes; versão 2 usa o byte 13 para delta.
Arquivos versão 1 continuam aceitos, com delta padrão 2. O CRC32 permanece
sobre os bytes 0–23. Firmware antigo não lê o novo arquivo versão 2.

## Filtro e perda

Cada pixel precisa de pelo menos dois dos oito vizinhos compatíveis com ele
(`MIN_PIXELS=3`, incluindo o próprio pixel). Compatibilidade usa a tolerância
efetiva. Sem suporte, o pixel recebe a mediana dos vizinhos disponíveis;
para quantidade par usa a mediana superior. Nas bordas não há padding.
O filtro lê exclusivamente a entrada original e escreve em outro buffer.
Isso preserva o interior de diagonais finas, mas pode remover pontas e
detalhes pequenos. A qualidade para detecção de linha precisa de validação
com imagens da aplicação.

O agrupamento percorre linhas da esquerda para a direita, de cima para baixo,
comparando com uma referência fixa por região. A referência não muda quando
uma sequência longa precisa ser dividida em registros. RAW e RLE representam
a mesma imagem filtrada e agrupada. Empates de tamanho usam RAW.

## Formato de `camera.matrix`, schema 2

A versão de schema BTP é 2 apenas para `camera.matrix` (0x0202);
`camera.status` continua na versão 1. Após o prefixo BTP de versão (u16 LE):

| Byte | Conteúdo |
| --- | --- |
| 0 | Linhas |
| 1 | Colunas |
| 2 | Depth: 1, 2, 4 ou 8 |
| 3 | Encoding: 0=RAW, 1=RLE |
| 4… | Dados |

RAW mantém pixels MSB-first e alinhamento de cada linha em um novo byte,
com bits de padding zerados. RLE ignora esse padding e pode atravessar linhas:

| Depth | Registro | Comprimento |
| --- | --- | --- |
| 1 | VCCCCCCC | 1–128 |
| 2 | VVCCCCCC | 1–64 |
| 4 | VVVVCCCC | 1–16 |
| 8 | byte de valor, byte de contador | 1–256 |

O contador armazena comprimento menos um. A soma dos comprimentos precisa
ser exatamente linhas × colunas. O receptor rejeita formatos desconhecidos,
dados truncados, excesso e falta de pixels. Delta não precisa viajar com a
imagem: seus efeitos já estão nos valores transmitidos.

60×60 em 4 bits possui 1.800 bytes RAW. Uma imagem uniforme usa 225 bytes RLE
(229 bytes com o cabeçalho da matriz). Não é uma taxa garantida para imagens
reais. A câmera usa dois buffers estáticos de 14.400 bytes, fora da pilha,
para amostragem e filtragem, sem alocação por frame.

O TraceView atualizado aceita schema 1 (RAW legado) e schema 2. Atualize-o
para visualizar o novo firmware. A fragmentação e o encaminhamento pelo
dongle continuam sendo responsabilidade do transporte BTP existente.

## Validação

```text
python scripts/test_camera_matrix.py --btp ../BTP
pio run -e esp32cam
```

O teste nativo aceita `--compiler` para escolher um compilador C++17. Cobre
as quatro profundidades, contadores, padding, filtro, tolerância, fallback,
limites de buffers e migração de configuração. O TraceView possui testes
correspondentes em `test_textboardwidget` e `test_telemetryfieldrouter`.

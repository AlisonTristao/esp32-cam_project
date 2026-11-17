# Compressão adaptativa da câmera

A captura aplica crop, reamostragem por `size` e quantização por `depth`
(em 1 bit, opcionalmente com threshold adaptativo). Em seguida seleciona
RAW/RLE sem nenhuma perda: os bytes transmitidos reconstroem exatamente a
grade quantizada, sem filtro de ruído nem agrupamento por tolerância.
A tentativa de compressão está sempre ativa, para todas as profundidades.

## Configuração

60×60 em 4 bits é o padrão de fábrica (`DEFAULT_MATRIX_SIZE`/
`DEFAULT_BITS_PER_PIXEL` em `Camera.cpp`). Para voltar a essa configuração
depois de mexer nos parâmetros:

```text
cam set_size 60
cam set_depth 4
cam status
cam save
```

Se a configuração atual de size/depth impedir uma transição, reduza primeiro
`size`. O limite continua calculado pelo pior caso RAW: o corpo completo deve
caber em 1.900 bytes, incluindo o cabeçalho de quatro bytes.

`cam status` mostra RAW/encoded bytes do último frame processado, incluindo
o cabeçalho da matriz, e o formato escolhido. Esses valores são atualizados
quando há assinatura ativa de `camera.matrix`; não incluem BTP, criptografia
ou cabeçalhos de rádio e não comprovam entrega do frame.

`cam save`/`cam load` persistem threshold/size/depth/crop/adaptive no SD.
`camera.cfg` permanece com 28 bytes; versão 3 usa o byte 13 para
`adaptive_threshold`. Arquivos versão 1 e 2 continuam aceitos, sempre com
adaptive_threshold=0 (o byte 13 dessas versões não tinha esse significado).
O CRC32 permanece sobre os bytes 0–23. Firmware antigo não lê o novo arquivo
versão 3.

## Threshold/contraste adaptativo

`cam set_adaptive 1` liga o mesmo princípio para as duas famílias de depth:
em vez de um "preto" e "branco" fixos, cada frame usa o próprio pixel mais
escuro e o mais claro amostrados. `cam adaptive` mostra se está ligado;
`cam set_adaptive 0` volta ao comportamento fixo de sempre.

Em `depth=1`, cada pixel amostrado vira `0` ou `1` comparando com um
threshold: pixel mais escuro que o threshold vira `1` (bit marcado), o
restante vira `0`. Isso casa com a convenção do TraceView de desenhar bit
marcado como preto (`textboardwidget.cpp`, `levelOf`/`decodeMatrix`), então
uma linha ou área escura na cena aparece escura na tela, em vez de sair como
o negativo da imagem. Por padrão o threshold é o valor fixo de
`cam set_threshold` (0–255). Com o adaptativo ligado, o firmware ignora esse
valor fixo e calcula, a cada frame, o ponto médio entre o pixel mais escuro
e o mais claro amostrados naquele frame (`(min + max) / 2`) -- assim o corte
preto/branco acompanha a exposição/iluminação atual em vez de precisar de
um ajuste manual toda vez que a cena muda.

Em `depth` 2/4/8, o padrão é truncar os bits mais significativos do pixel
(escala de cinza linear assumindo a faixa 0–255 inteira). Com o adaptativo
ligado, o firmware em vez disso estica linearmente o intervalo
`[mínimo, máximo]` amostrado naquele frame para `[0, 2^depth - 1]` -- uma
cena de baixo contraste (iluminação fraca, por exemplo, deixando os pixels
todos numa faixa estreita) passa a usar todos os níveis de cinza
disponíveis, em vez de ficar concentrada em um ou dois. A troca é
brilho absoluto entre frames: o mesmo objeto físico pode sair com um nível
de cinza diferente de um frame para outro se a faixa da cena mudar. Um
frame perfeitamente uniforme (sem variação) cai de volta na truncagem
simples em vez de dividir por zero.

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
dados truncados, excesso e falta de pixels. RAW e RLE reconstroem exatamente
a mesma grade quantizada -- nenhum valor é alterado antes de transmitir.

60×60 em 4 bits possui 1.800 bytes RAW. Uma imagem uniforme usa 225 bytes RLE
(229 bytes com o cabeçalho da matriz). Não é uma taxa garantida para imagens
reais. A câmera usa dois buffers estáticos de 14.400 bytes, fora da pilha,
para amostragem (pré-quantização) e para a grade quantizada, sem alocação
por frame.

O TraceView atualizado aceita schema 1 (RAW legado) e schema 2. Atualize-o
para visualizar o novo firmware. A fragmentação e o encaminhamento pelo
dongle continuam sendo responsabilidade do transporte BTP existente.

## Validação

```text
python scripts/test_camera_matrix.py --btp ../BTP
pio run -e esp32cam
```

O teste nativo aceita `--compiler` para escolher um compilador C++17. Cobre
as quatro profundidades, contadores, padding, fallback, limites de buffers
e migração de configuração. O TraceView possui testes
correspondentes em `test_textboardwidget` e `test_telemetryfieldrouter`.

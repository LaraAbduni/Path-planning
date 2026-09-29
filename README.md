# Projeto Intermediário de Inteligência Artificial e Robótica — Path Planning

Por **Arthur Alexandre, Giovanna Scalco e Lara Abduni**.

## Sobre o projeto

Este projeto implementa um algoritmo de *path planning* para um robô utilizando o algoritmo **A\***. A partir de um mapa no formato `.pgm`, de uma posição inicial e de uma posição objetivo, o programa calcula um caminho seguro em direção ao destino.

O mapa pode conter regiões livres, obstáculos e áreas ainda desconhecidas pelo robô. Como o mapeamento do ambiente é incompleto, o robô deve avançar somente pela região conhecida. Ao alcançar a borda dessa região, ele deve parar, perceber novas partes do ambiente e recalcular o caminho.

## Objetivo

O objetivo é gerar um caminho que:

- conduza o robô da posição inicial em direção ao objetivo;
- não atravesse obstáculos ou regiões consideradas inseguras;
- mantenha uma distância de segurança das paredes;
- avance apenas até o limite da região conhecida;
- possa ser recalculado conforme novas partes do mapa forem descobertas.

## Como funciona

O programa realiza as seguintes etapas:

1. Carrega o mapa em escala de cinza no formato `.pgm`.
2. Classifica as células do mapa como obstáculo, região desconhecida ou espaço livre.
3. Aumenta a margem ao redor dos obstáculos para impedir que o robô passe muito perto das paredes.
4. Calcula um campo potencial baseado na distância até os obstáculos.
5. Executa o algoritmo A\*, considerando tanto a distância percorrida quanto o custo de proximidade das paredes.
6. Reconstrói o caminho encontrado.
7. Interrompe o caminho no primeiro ponto pertencente a uma região desconhecida.
8. Simplifica o resultado, mantendo o ponto inicial, as mudanças de direção e o ponto final conhecido.

No mapa processado, os valores possuem o seguinte significado:

| Valor | Significado |
| ---: | --- |
| `0` | Obstáculo |
| `128` | Região desconhecida |
| `255` | Região livre |

## Principais parâmetros

Ao criar o objeto `AStarPathfinder`, podem ser configurados:

- `start`: coordenada inicial no formato `(linha, coluna)`;
- `goal`: coordenada objetivo no formato `(linha, coluna)`;
- `wall_influence`: peso da penalização aplicada perto das paredes;
- `buffer_factor`: tamanho da margem de segurança ao redor dos obstáculos.

Exemplo:

```python
astar = AStarPathfinder(
    map_array,
    (60, 20),
    (60, 120),
    wall_influence=10.0,
    buffer_factor=3.0
)
```



## Como executar

Instale o **Python 3** e, no terminal, execute:

```bash
pip install numpy opencv-python matplotlib scipy
```

Depois, coloque o mapa `.pgm` no caminho definido no código e rode o arquivo:

```bash
python astar.py
```

O programa calculará o caminho e exibirá a visualização do resultado.

Ao final da execução, o programa retorna o caminho simplificado e pode exibir uma visualização contendo:

- o ponto inicial em verde;
- o objetivo em azul;
- o caminho completo em magenta;
- o caminho simplificado em vermelho tracejado.

## Vídeo do robô

[Link do vídeo do robô funcionando](https://youtube.com/shorts/QucrSaeLNf0?si=Ysff48Me1zvZ4ROu)
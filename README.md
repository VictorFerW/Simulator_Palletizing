# Palletizing Digital Twin

Simulador 3D no estilo Factory IO, focado em células de paletização com o robô FANUC M-410iC/185, feito em Rust (Bevy + egui). Ele importa o projeto do TIA Portal V18 (`.ap18`), permite escolher as tabelas de tags e os DBs e conversar com o CLP (ou o PLCSIM Advanced) por S7.

## Executar

```powershell
cargo run --release
```

Na primeira execução, o programa abre uma célula padrão que já funciona sem CLP:

- **Produto:** emissor → esteira de correia → Mesa 2 estágios → FANUC M-410iC/185.
- **Pallet:** magazine de pallets → TDP → TDP de paletização (o robô empilha no pallet parado no batente) → TDP de saída → removedor.

## Equipamentos

| Equipamento | Baseado em | Comportamento |
|---|---|---|
| Mesa antecipada 2 estágios | Selgron "Mesa pré antecipada L=1200" + FB "Mesa Antecipada Roletada" | Roletada (bloqueio de entrada, motor, empurrador lateral Pos1 → Pos2/Pos3) + coleta |
| Mesa pré-forma 3 estágios | Selgron "Mesa pré-forma 3 estágios" + FB "Mesa 3 Estágios" | Roletada com garra (sobe/desce/avança/recua), pré-formadora com empurrador, bloqueios de entrada/meio/coleta (motor compartilhado) + coleta |
| TDP | Selgron "TDP 1560 x 1280 x 120" | Rolos Ø76,2 a cada 120 mm, topo a 500 mm, 15 m/min; transporta pallets |
| TDP de paletização | TDP + batente de posicionamento | Segura o pallet no batente enquanto o robô paletiza. Com **lógica interna** ligada, traz o pallet vazio, segura e libera quando completa; o que vier do CLP tem prioridade |
| Magazine de pallets | — | Coloca pallets vazios (PBR 1000 × 1200) sobre o TDP |
| Removedor | — | Remove caixas e pallets (com a carga) que param na sua área |
| Carro cheio giratório (CCH) | Selgron "MOD CARRO ALIMENT ROTATIVO" + FBs "Carro Cheio Movimento" e "Carro Cheio TDP" | Carro sobre trilho com TDP redondo (Ø2054, rolos 1829 × 1380 a 514 mm) num rolamento de giro. Busca os pallets completos nas linhas e entrega girados |
| Elevador de pallets (alimentador) | Selgron "MOD ALIMENTADOR DE PALETE" + FB do elevador | Quadro de quatro colunas sobre um TDP, com garras laterais num carro vertical e a pilha de pallets vazios. Solta um pallet por vez no TDP |
| Colocador de papelão (CP) | Selgron "MOD COLOCADOR PAPELAO C2370" + FB "CP" | Pórtico com carro entre o depósito de papelão e a entrega (1887 mm), cilindro vertical e cabeça de ventosas. Pode ficar sobre um TDP normal ou sobre uma parada do CVZ |
| Carro de pallet vazio (CVZ) | FB "CVZ" | Carro sobre trilho com TDP que transporta perpendicular ao trilho; o pallet anda com o carro. Inversor (`LigarDrive`, `LiberarFreio`, `Velocidade` em mm/s) e sensor de distância (`PosicaoAtual` em mm) |

As duas mesas terminam na **mesa área de coleta** (FB "Mesa Antecipada Coleta"): motor e batente próprios, sensores Coleta1/Coleta2. Ela recebe o par de fardos, recolhe o batente e dá `Autoriza_Pick`; o robô pega o par inteiro e responde `Robo_Fardos_Coletados`.

Os sinais das mesas têm os **mesmos nomes das variáveis dos FBs** (`Bloqueio_Entrada`, `Sensor_RoletadaPos1`, `GarraAvancar`, `SensorEtapa2Pac3`…), então o CLP real comanda os atuadores e lê os sensores simulados. Sem CLP, a opção **Lógica interna** executa um port dos FBs em Rust (`src/mesa_logic.rs`, mesmos passos e temporizadores) no modo automático, para camadas normais. Os modos última camada ímpar, 10x1 e Fardo10Kg (3/4 fardos por coleta) não estão portados; com o CLP conectado eles funcionam, porque quem executa é o CLP.

As caixas colocadas pelo robô passam a fazer parte do pallet e andam junto com ele pelos TDPs.

## Editor

| Ação | Como fazer |
|---|---|
| Adicionar | Painel **Catálogo**: esteira, Mesa 2 estágios, Mesa pré-forma 3 estágios, TDP, TDP de paletização, robô FANUC M-410iC/185 ou KUKA KR 180 R3200-2 PA, emissor, magazine de pallets, removedor, sensor fotoelétrico |
| Selecionar | Clique no objeto ou na lista **Cena** |
| Seleção múltipla | `Ctrl`+clique soma ou tira itens; arrastar com o botão esquerdo num espaço vazio desenha um retângulo de seleção (`Ctrl` para somar à seleção atual) |
| Mover | Arraste qualquer item selecionado: o grupo inteiro se move (grade de 0,1 m; segure `Alt` para ignorar a grade). Setas: 0,1 m (`Shift`: 0,5 m). `PgUp`/`PgDn`: altura |
| Rotacionar | `R` / `Shift+R`: ±90° · `Q` / `E`: ±15° (um grupo gira em torno do seu centro) · ou o campo Rotação em Propriedades |
| Deletar | `Del` (todos os selecionados), o botão 🗑 da lista ou **Excluir** |
| Duplicar | `Ctrl+D` (todos os selecionados) |
| Câmera | Botão direito: orbitar · botão do meio ou `Shift`+direito: deslocar · roda: zoom · `F`: focar no selecionado |

Com itens selecionados, segure **V** e mova o mouse para cima ou para baixo para subir ou descer a seleção (em passos da grade quando ela está ligada; **Alt** desliga o encaixe).

Em **Propriedades** você ajusta comprimento, largura, altura e velocidade dos transportadores; no robô, a mesa de coleta, o TDP de paletização e o override de velocidade; no TDP de paletização, o número de camadas, o intertravamento (camadas ímpares giradas 90°) e a lógica interna; no emissor, o intervalo e o tamanho da caixa. Uma esteira que entra de lado em outra (como a Linha 1B na Linha 1) segura o fardo no fim até o trecho da outra ficar livre, e as guias laterais (largura da esteira) mantêm o fardo na faixa de entrada da mesa.

**Reiniciar** (barra de ferramentas) tira todas as caixas e pallets e volta cada equipamento ao primeiro passo do seu FB: robôs, mesas, elevadores, colocadores de papelão (CP) e carros (CVZ/CCH, que voltam ao início do trilho), mesmo no meio de um ciclo.

### Projetos

Ao abrir o simulador aparece a **tela inicial** com os projetos:

- **Novo projeto (vazio)** ou **Novo com a célula padrão**: dê um nome; o projeto já é criado e salvo na pasta de projetos.
- Cada projeto salvo aparece num cartão com a **prévia do layout vista de cima** (robô em laranja, mesas em azul, TDPs, carros CVZ/CCH em amarelo, emissores em verde, removedores em vermelho), o número de equipamentos e quando foi alterado. Clique na prévia ou em **Abrir** para abrir.
- **Excluir** pede confirmação e apaga o arquivo. Para um arquivo de fora da pasta, também dá para só tirá-lo da lista.
- **Abrir outro arquivo...** abre um layout salvo em outro lugar; ele passa a aparecer na lista como recente.

A pasta é `Documentos\Simulador de Paletização\Projetos` (ou a definida em `PDT_PROJECTS`). No menu **Arquivo**: **Projetos (tela inicial)** volta a essa tela (com **Voltar ao projeto aberto**), **Salvar** (`Ctrl+S`) grava no arquivo do projeto aberto, **Salvar como...** escolhe outro arquivo, e **Abrir arquivo...** abre qualquer layout. O nome do projeto aberto aparece na barra de ferramentas.

O arquivo do projeto (JSON) grava a célula inteira em JSON: equipamentos, vínculos de tags, projeto TIA importado e configurações do CLP.

## Robôs: FANUC M-410iC/185 e KUKA KR 180 R3200-2 PA

O robô pode ficar acima do piso: com **Altura (Y)** maior que zero, ele é desenhado sobre um pedestal, e o braço trabalha abaixo da própria base até o limite da folha de dados (FANUC −561 mm, KUKA −742 mm). O piso de verdade continua sendo o limite: o robô só reporta "abaixo do piso" para pontos abaixo do chão da célula. Com o robô a 0,8 m, por exemplo, o topo de um pallet num TDP de 0,5 m fica 0,15 m abaixo da base e é alcançado normalmente.

Há dois modelos no catálogo: **Robô FANUC M-410iC/185** e **Robô KUKA KR 180 R3200-2 PA**. No painel do robô, **Modelo** troca o modelo de um robô já colocado (ele volta à posição de repouso do novo modelo). Cada modelo tem a sua cinemática, faixas, velocidades, envelope e modelo 3D do fabricante. O cabeçote, o programa de paletização, as rotas e o log de falhas são os mesmos.

### FANUC M-410iC/185

O modelo é paramétrico e reproduz a cinemática de paletizador de 4 eixos: J1 é a base, J2/J3 formam o braço com paralelogramo que mantém o flange horizontal e J4 gira o punho. O alcance é de aproximadamente 3,1 m, como na folha de dados. O programa interno faz o papel da rotina de paletização do controlador: espera a caixa no fim do transportador, coleta, sobe até a altura segura, gira, deposita na posição do padrão, sobe de novo e repete. Quando o pallet fica completo, sinaliza **Pallet completo**.

O visual usa o **modelo do fabricante** (STEP SolidWorks M-410iC185, com pedestal), convertido para 12 peças em `assets/models/fanuc_m410ic185/*.glb`. Cada peça acompanha a sua articulação: base e pedestal fixos, carrossel J1, braço J2 (S302), alavanca e biela do J3 (S303/S304), braço J3 (S402), paralelogramo de nivelamento (S305/S401/S403), punho (S501) e flange J4 (S502). As medidas da cinemática foram tiradas do próprio modelo: J2 a 1,110 m do chão, offset 0,394 m, braços de 1,220 m e 1,306 m, e flange 0,244 m à frente e 0,159 m abaixo do pino do punho. Isso dá um alcance horizontal até o flange de 3,164 m (catálogo: 3143 mm). Se os arquivos não estiverem em `assets/`, o simulador usa o modelo paramétrico. Ao rodar o executável fora do `cargo run`, copie a pasta `assets` para junto do `.exe` ou defina `BEVY_ASSET_ROOT`.

### Cabeçote

### Dados reais (folha de dados FANUC)

| Item | Valor |
| --- | --- |
| Carga máxima no punho | 185 kg |
| Alcance (centro do flange) | 3143 mm |
| Repetibilidade | ±0,05 mm |
| Massa | 1600 kg (1330 kg sem pedestal) |
| Faixas de movimento | J1 360° (±180°), J2 144°, J3 136°, J4 720° (±360°) |
| Velocidades máximas | J1 140°/s, J2 140°/s, J3 140°/s, J4 305°/s |
| Momento admissível no J4 | 88 N·m |
| Espaço de trabalho do flange | de −561 mm a 2397 mm de altura (base pedestal, J2 a 1110 mm), raio interno de 1100 mm no piso, interferência traseira R725 |

A divisão das faixas de J2 e J3, que a folha de dados dá só como total, foi escolhida para reproduzir o desenho do envelope: J2 (braço inferior a partir da vertical, para a frente positivo) de −44° a +100° e J3 (braço superior a partir da horizontal) de −126° a +10°. Com esses limites, o topo, o fundo e o raio interno do envelope batem com o desenho (há um teste para isso). Uma posição fora dos limites é tratada como fora do alcance.

Os movimentos seguem as rotinas TP. As transferências são **J** (interpolação por eixos): a duração vem do eixo mais lento na sua velocidade máxima. As subidas e descidas são **L** (reta) nas velocidades das rotinas: PICK desce a 3000 mm/s e sobe a 1800 mm/s; PLACE desce a 2000 mm/s e sobe a 4000 mm/s, limitadas pelas velocidades dos eixos. Todos usam perfil trapezoidal com rampa de 0,35 s e são multiplicados pelo override (`OVERRIDE=R[104]`). O J1 não passa de ±180°: se a mesa e o pallet estiverem dos dois lados dessa emenda, o robô dá a volta longa, como o real. Por isso, gire a base do robô para que o 0° do J1 aponte para a área de trabalho (na célula padrão, −135°). O painel do robô mostra a posição e a velocidade de cada eixo contra os limites. Com o robô selecionado, o editor desenha o alcance, o raio interno, a interferência traseira e o contorno do espaço de trabalho no plano do braço.

A altura de segurança das transferências e o perch são limitados pela altura máxima que o braço alcança naquele raio. Perto do topo do envelope essa altura cai: com o cabeçote (435 mm até o quadro mais 295 mm de pinça), a base dos fardos do FANUC chega a ~1,65 m em 2,1 m de raio e ~1,18 m em 2,9 m. Se não houver folga para passar sobre o pallet, o robô para com uma falha no log, em vez de esticar o braço além dos limites.

A geometria do braço vem do modelo STEP do fabricante. Ele divide os mesmos 1945 mm um pouco diferente da folha de dados: 394/1306/244 mm contra 390/1300/255 mm.

### KUKA KR 180 R3200-2 PA

| Item | Valor (folha de dados 0000-361-137 V5.1) |
| --- | --- |
| Carga nominal / máxima | 180 kg / 230 kg |
| Alcance máximo | 3195 mm |
| Repetibilidade (ISO 9283) | ±0,07 mm |
| Eixos | 5 (A4 não existe) |
| Massa | ~1017 kg, base 754 × 754 mm, no piso |
| Faixas de movimento | A1 ±185°, A2 −140° a −5°, A3 0° a 165°, A5 −55° a 150°, A6 ±350° |
| Velocidades com carga nominal | A1 137°/s, A2 111°/s, A3 119°/s, A5 225°/s, A6 321°/s |

As medidas vêm do esqueleto que está no próprio STEP do fabricante: A2 a 645 mm do piso e 330 mm à frente do A1, braço de ligação (A2–A3) de 1350 mm, braço (A3–A5) de 1255 mm, e flange 265 mm à frente e 250 mm abaixo do A5. No KUKA, A2 = −90° e A3 = +90° com o braço de ligação na vertical e o braço na horizontal: o A3 é o ângulo entre os dois braços (e não a partir da horizontal, como o J3 do FANUC). O A5 mantém o flange na horizontal. Com as faixas do A2 e do A3 e o braço subindo no máximo 60° acima da horizontal (gráfico do espaço de trabalho), o envelope do centro do flange fica igual ao do STEP: alcance de 3195 mm a 513 mm de altura, topo a 2832 mm e fundo a −742 mm (há testes para isso). O painel mostra A1, A2, A3 e A6 na convenção KUKA. As velocidades de cada movimento são verificadas nesses eixos, então um movimento que só gira o braço (A3) é limitado pelos 119°/s do A3.

O modelo 3D (`assets/models/kuka_kr180_r3200_2_pa/*.glb`) foi separado do STEP em base fixa, coluna (A1), braço de ligação (A2), braço (A3), punho (A5) e flange (A6).

O programa segue as rotinas KRL da célula (`RSR0001`, `MovePick`, `MovePlace`, `MovePerch`, `MoveHome`):

- **MovePick:** sobe em SLIN a 3,0 m/s até a altura segura (no máximo Z 2000), vai em SPTP 100 % até acima da coleta e desce em SLIN (3,0 até o ponto de aproximação e 2,5 m/s até a pega). O simulador desce direto a 2,5 m/s.
- **MovePlace:** sobe em SLIN a 3,0 m/s, vai em SPTP 100 % até acima do depósito e desce em SLIN (2,5 até a aproximação e 3,0 até o depósito). O simulador desce a 2,5 m/s. No segundo depósito da mesma pega (`RoboSegundoPlace`), sobe só o necessário para passar ao lado.
- As transferências ficam abaixo de 2,0 m, a não ser que a camada exija mais. O status mostra a rotina em execução (por exemplo, "MovePick: indo para a coleta").

Ainda não estão simulados: o desvio por X 1500 / Y ±1200 quando a coleta e o depósito ficam em lados opostos, a aproximação contínua (C_SPL, `$APO.CDIS`) e os dados de aproximação enviados pelo CLP (`DadosXAprox`, `DadosDesviaInfeed`…).

### Várias mesas, vários TDPs e vários robôs

Em **Coletar de** e **Paletizar em** marque uma ou mais mesas/esteiras e um ou mais TDPs de paletização; sem nenhum marcado, o robô usa todos ao alcance. As mesas são atendidas em rodízio, e o TDP com o pallet mais adiantado vem primeiro, então os pallets são completados um de cada vez (quando um sai, o robô passa para o outro).

Em **Rotas no alcance**, o robô lista sozinho os TDPs de paletização e as mesas que alcança. Para cada TDP, escolha de quais mesas ele recebe (por exemplo, TDP A recebe da Mesa B e TDP B recebe da Mesa A); sem escolha, o TDP recebe de todas. Se um grupo de fardos chega girado 90° em relação ao do pallet (por exemplo, vindo de uma esteira lateral), o robô o gira para casar com a posição.

### Log de falhas

Quando o robô não consegue chegar a uma posição, ele para com o status **Falha: posição fora do alcance** (em vermelho) e registra o motivo em **Log de falhas**, no painel do robô (as 20 mais recentes, a mais nova primeiro; a mesma falha só é registrada de novo depois que muda). Cada linha diz qual ponto falhou e por quê:

- **Coleta em Mesa_X** ou **depósito no TDP_X (posição N do padrão)**: o raio e a altura do TCP e o motivo, por exemplo "longe demais: 3190 mm do eixo J2 até o punho, o braço estica até 2526 mm", "J3 precisaria de 14° (faixa −126° a 10°)", "abaixo do piso" ou "nesse raio o TCP sobe só até 1,39 m". Cada falha entra no log uma vez enquanto dura; se ela some e volta, entra de novo.
- **Altura sobre o TDP_X**: os fardos carregados (o TCP fica na base deles) precisam passar 5 cm acima do topo da camada em montagem, e o braço não sobe tanto naquele raio. A mensagem dá a altura necessária, a altura máxima e o raio.

Sobre o pallet, o robô passa a 15 cm acima da camada em montagem (nunca acima do que alcança naquele raio). Na coleta, que costuma ficar perto da base, ele sobe só até onde alcança ali. Com `PDT_TRACE=1`, as falhas também saem no terminal como `FALHA ...`.

Na célula padrão, os TDPs de paletização usam 3 camadas de fardos de 250 mm: com 4, a 4ª camada dos TDPs 1 e 2 fica acima do que o M-410iC alcança a 2,06 m de raio, e o log mostra essa falha.

Vários robôs podem dividir mesas e TDPs. Cada robô reserva a mesa do planejamento até levantar os fardos e o TDP até terminar a deposição; quem está esperando um equipamento ocupado é o próximo a usá-lo, então os robôs se revezam. A colisão entre os braços não é simulada.

### Perch

Como na rotina MOVE_PERCH, quando não há ciclo para começar o robô vai para a posição de espera em cima da coleta da próxima mesa, na altura de infeed (topo da mesa + fardo + 0,35 m), com o cabeçote alinhado à mesa, e espera ali o Autoriza_Pick ou o pallet.

O robô usa o cabeçote Selgron "MOD CAB 4 FARDOS" (0323P01-010000): quadro de 1234 × 500 mm com acoplamento 435 mm acima da base (os fardos ficam no fundo da garra, entre as pontas das pinças: o TCP é a ponta das pinças, na base dos fardos), divisória central de 229 mm e duas pinças laterais de 295 mm acionadas pelos cilindros CIL 40-420. As pinças fecham sobre o grupo coletado e abrem ao depositar.

## Colocador de papelão (CP)

O carro do CP corre entre a posição de **pega**, sobre o depósito com a pilha de papelões, e a de **entrega** (círculo amarelo quando o CP está selecionado), onde o centro do pallet deve parar. A cabeça de ventosas é flutuante: o `SensorPapelaoInferior` fica ligado com a cabeça solta e sem papelão, e o `SensorPapelaoSuperior` liga quando ela é pressionada contra a pilha ou o pallet. A folha aplicada fica sobre o pallet, anda com ele, e o robô empilha em cima dela.

- **Sobre um TDP normal:** um pallet sem papelão que chega sob a entrega é segurado ali (como o TDP parando no sensor) até o fim do ciclo, e a fila atrás dele espera.
- **Como parada do CVZ:** marque a caixa **CP** numa das paradas do carro. Depois de receber o pallet na coleta, o carro vai primeiro até essa parada, espera o papelão e só então segue para o destino.

Sem CLP, a **lógica interna** porta os passos automáticos do FB CP (0–2, 100–117, 200/201, 400): desce na pilha, liga o vácuo ao perder os dois sensores, para no sensor superior, estabiliza, sobe, confere o sensor inferior (sem papelão → passo 400), vai à entrega, desce até o sensor superior, desliga o vácuo, sobe e volta. A opção **Aplicar 2x papelão** repete a aplicação. O botão **Reabastecer depósito** repõe a pilha. A escolha "aplicar ou não" por ID de linha do FB vem do CLP; internamente vale a opção **Aplicar papelão**.

## Padrões de paletização

O botão **Padrões de paletização** (barra de cima, ou **Escolher padrão…** no TDP de paletização) abre a tela de formatos. Nela você informa:

- **Produto:** comprimento, largura, altura e peso. O botão **usar fardo da mesa** copia o fardo da primeira mesa.
- **Cabeçote do robô:** produtos na garra (1 ou 2, um em cada pinça), lado a lado ou em linha. Cada posição do formato é um produto.
- **Pallet:** os mais usados (PBR 1000 × 1200, Europeu 800 × 1200, Industrial 1200 × 1000, Americano 40 × 48", 1100 × 1100, 1100 × 1300, 1140 × 1140) ou personalizado, mais a sobra permitida por lado e a folga entre unidades.
- **Limites:** altura máxima com o pallet, peso máximo e camadas (0 = o máximo permitido).

A tela gera os formatos mais usados e mostra as camadas ímpares e pares vistas de cima, numeradas na ordem do robô:

- **Coluna:** todas as unidades no mesmo sentido, camadas iguais.
- **Coluna cruzada:** as camadas pares giradas 90°. Aparece quando o número por camada é o mesmo nos dois sentidos.
- **Amarração (tijolo):** um bloco no comprimento e outro atravessado, com as camadas pares espelhadas para desencontrar as juntas (por exemplo, os 5 sacos de 600 × 400 em 1200 × 1000).
- **Catavento (pinwheel):** quatro blocos girando em volta do centro.

No fim da tela, **Produto nos emissores e mesas** lista os emissores de caixas e as mesas da célula com o tamanho atual. Marque os que devem receber o produto informado e clique em **Atualizar tamanho do produto**. Os emissores passam a criar caixas desse tamanho, e as mesas refazem as pistas e o empurrador para o novo fardo. As caixas já criadas continuam com o tamanho antigo.

### Pegas com a garra de dois lados

Ao gerar o formato, a tela calcula a sequência de pegas. As vistas mostram o número da pega em cada posição: o mesmo número nas duas posições de um par, e "a/b" quando a pega deposita um produto de cada vez.

- **Par:** duas posições vizinhas no sentido em que os produtos ficam na garra. O robô deposita os dois juntos e abre as duas pinças.
- **Dois depósitos simples:** posições que não formam par. O robô abre só a pinça do produto daquela posição, sobe até as pinças e o outro produto passarem acima do que já foi colocado, gira e deposita o outro.
- **Camada ímpar:** em cada camada vêm primeiro as posições que não formam par, depois os pares. A que sobrar vai junto com o primeiro depósito simples da camada seguinte, e o robô sobe para depositá-lo lá. Com 5 por camada (2 atravessados e 3 no comprimento), a sequência fica: pega 1 = as duas atravessadas, uma de cada vez; pega 2 = o par; pega 3 = a última da camada mais a primeira da camada de cima.
- **Último produto do pallet:** quando sobra um só, o robô pede à mesa um produto único. Entra só um fardo, o empurrador empurra, a roletada libera para a coleta e o cabeçote pega com a pinça desse lado.

No depósito simples o giro do punho é exato (só voltas inteiras, dentro de ±360° do J4), porque meia volta colocaria o outro produto na posição. As pinças encostam os dois fardos do par um no outro ao fechar. Com `PDT_TRACE`, o simulador informa qual pinça abre em cada depósito e avisa se um fardo da garra encostar num fardo do pallet.

### Receita 10x1 (mesa 3 estágios)

Na mesa 3 estágios, a opção **Receita 10x1** (o `Fardo10Kg` do FB) troca a lógica normal pela do 10x1, portada dos passos 100–300 da roletada e 100–202 da pré-formadora. A mesa passa a ter quatro posições lado a lado, da pista de entrada até o batente físico da pré-formadora:

1. **Roletada:** entra um fardo. Se a pré-formadora pede um, ele segue direto. Se pede dois, a garra empurra a fileira, entra outro, a garra empurra de novo e os dois seguem.
2. **Pré-formadora:** o primeiro fardo vai até o batente pelo empurrador; o cilindro alivia e chegam os dois da roletada, formando um grupo de 3.
3. **Coleta:** com o grupo de 3 (como no FB), os 3 vão para a coleta.
4. **Grupo de 4 (como na máquina):** a roletada acumula 3 (entra um, a garra empurra, entra outro, a garra empurra de novo, entra o terceiro) e manda os 3 juntos. O empurrador leva a fileira até a **guia física** da pré-formadora, o 4º fardo vem reto e os 4 vão para a coleta.

O empurrador da receita 10x1 empurra a fileira inteira e para quando ela encosta na guia. O tamanho de cada grupo (`FardosPonto`) vem da sequência de pegas do padrão aplicado, que o robô passa para a mesa; a mesa a percorre grupo a grupo.

Na tela de padrões, marque **receita 10x1** no cabeçote. As pegas passam a seguir o ciclo **4 → 3 → 3**: o robô pega os 4 e deposita 2 numa ponta e 2 na outra (abre um lado do cabeçote de cada vez), depois deposita os 3 de uma vez, e de novo 3. A tela põe primeiro os formatos que fecham esse ciclo, como 10 por camada em 2 fileiras de 5 com o fardo atravessado, e avisa quando um formato não fecha. O robô avisa a mesa quantos fardos a próxima pega precisa. Se chegar um número diferente, ele deposita os fardos um a um nas próximas posições.

Cada formato mostra unidades por camada, camadas e total, altura e peso (e qual limite parou as camadas), a ocupação da área do pallet, a sobra e se as camadas são amarradas. Para o TDP escolhido, a tela mostra a altura do topo do pallet completo e avisa quando ela passa do alcance do robô com o cabeçote. **Aplicar ao TDP de paletização** grava o formato e as camadas no TDP (e no layout salvo), e o robô passa a seguir a ordem e a orientação do formato. **Usar automático** volta à grade simples.

## Carro cheio giratório (CCH)

Coloque o trilho ao longo das linhas, com os TDPs de paletização descarregando no carro (o TDP do carro fica atravessado no trilho para receber). As paradas são lidas pelo sensor de distância: **Espera** (ID 10), **Entrega** (ID 20) e **Linha A, B...** (IDs 100, 200...). O ângulo **Giro receber → entregar** define para onde o pallet sai na entrega: 90° ao longo do trilho, 180° para o outro lado. O TDP é redondo, com os rolos das pontas cortados no círculo, para girar sem bater nos TDPs vizinhos.

Sem CLP, a lógica interna porta os dois FBs:

- **Movimento:** com uma linha pedindo (`ID_Busca`), vai até ela; com pallet no centro, vai para a entrega; sem nada, vai para a espera (e desiste da espera se surgir um pedido). O posicionamento segue os passos 200–203: velocidade rápida (com pallet ou vazio), redução proporcional, velocidade lenta nos últimos 50 mm, tolerância de 5 mm, freio só com a velocidade zerada.
- **TDP:** gira para receber, recebe até centralizar (Sensor 3 com Sensor 2 e sem o Sensor 1; o Sensor 4 indica fora do centro), gira para entregar a caminho da entrega, transfere e volta a girar para receber.

`ID_Busca` vem do rastreamento do CLP. Internamente, é a linha que espera há mais tempo com um pallet completo no TDP que descarrega no carro.

## Elevador de pallets (alimentador)

Coloque o elevador centrado sobre um TDP (a **Altura do TDP** é a do topo dos roletes). A pilha fica levantada pelas garras no sensor **superior**. Em cada ciclo do FB (passos 100–111): com o TDP pronto para receber e sem pallet, desce até o sensor **inferior**, abre a garra (a pilha apoia no TDP), sobe até o sensor **intermediário** (uma altura de pallet), fecha no segundo pallet, espera o **TDP posterior pronto** e sobe a pilha até o superior. O pallet de baixo fica no TDP e segue; o FimDoCiclo volta a zero quando ele sai do sensor. Os passos de reconhecimento (0–43) e as falhas por tempo (400–402) também estão portados.

Sem CLP, `TDPProntoReceber` vem do TDP sob o elevador (sem pallet sobre ele) e `TDPosteriorProntoReceber` fica ligado. Com a pilha vazia, o elevador espera embaixo com a luz **SEM PALLET NA PILHA**. **Reabastecer pilha** repõe a pilha; com **Abastecer automaticamente**, uma pilha cheia é colocada 3 s depois que o elevador fica vazio embaixo.

## Carro de pallet vazio (CVZ)

O trilho fica ao longo do X local do equipamento e o TDP do carro transporta ao longo do Z local: recebe de um lado do trilho e entrega do outro. O sensor de distância fica no início do trilho e mede o centro do carro em mm. As paradas (coleta, posição A, B, …) são configuradas em Propriedades.

Sem CLP, a **lógica interna** porta o posicionamento do FB CVZ (passo 201): velocidade nominal, redução proporcional a partir de 500 mm, velocidade de precisão nos últimos 50 mm, desliga o inversor na tolerância de 5 mm e trava o freio quando a velocidade real chega a zero. O ciclo interno é simples: recebe o pallet na coleta, leva às posições A, B… em sequência, descarrega e volta. A escolha do destino do FB real depende dos IDs de rastreamento dos pallets; com o CLP conectado, ela vem do CLP.


O CVZ pode ter várias **coletas**: marque a caixa **Coleta** nas paradas onde ele recebe pallets vazios. O carro vai à coleta que tem pallet esperando no fim do TDP, e as coletas com pallet se revezam. O lado é tirado do layout: se o TDP que entrega está do outro lado do carro, o TDP do carro roda ao contrário até o pallet encostar no fim. Na descarga, ele roda para o lado onde está o TDP que recebe.
## Sinais

| Equipamento | Entradas (CLP escreve) | Saídas (simulador escreve) |
|---|---|---|
| Esteira | Motor ligado, Sentido reverso | — |
| Mesa antecipada 2 estágios | Motor, Bloqueio_Entrada, Bloqueio_Coleta, EmpurradorAv/Rc, Motor_Coleta, Batente_Coleta | Sensor_EsteiraFim, Sensor_AntesBloqueio, Sensor_AposBloqueio, Sensor_RoletadaPos1/2/3, Sensor_EmpurradorAv/Rc, Sensor_Coleta1/2 |
| Mesa pré-forma 3 estágios | Motor, BloqueioEntrada/Meio/Coleta, GarraSobe/Desce/Avancar/Recuar, EmpurradorAvancar/Recuar, Motor_Coleta, Batente_Coleta | SensorEsteira, SensorEtapa1Pac1ProxBloq, SensorEtapa1Pac1/2/3, SensorGarraLevantada/Recolhida/Av/Rec, SensorEtapa2Pac1/2/3, SensorEmpurradorAv/Rec, Sensor_Coleta1/2 |
| TDP | Motor ligado, Sentido reverso | Pallet no fim do TDP |
| TDP de paletização | Motor ligado, Batente de posicionamento | Pallet em posição, Pallet completo, Caixas no pallet (Int) |
| FANUC M-410iC/185 | Liberar ciclo, Autoriza_Pick | Em ciclo, Robo_Fardos_Coletados, Pallet completo, Caixas no pallet (Int) |
| Emissor / Magazine / Removedor | Emitir / Liberar pallet / Remover | — |
| Carro cheio giratório | Motor, LiberarFreio, Velocidade, MotorTDP, LiberarFreioTDP, VelocidadeTDP, MotorGiro, LiberarFreioGiro, VelocidadeGiro | PosicaoAtual, DriverVelocidade, DriverFalha, FimDeCurso, Sensor1–4, Giro_Sensor_PosReceber, Giro_Sensor_PosEntregar, VelocidadeAtualTDP, VelocidadeAtualGiro |
| Elevador de pallets | MotorSobe, MotorDesce, Garra_Abrir, Garra_Fechar, TDPProntoReceber, TDPosteriorProntoReceber | SensorPosInferior, SensorPosInferiorMec, SensorPosIntermediario, SensorPosSuperior, SensorPosSuperiorMec, SensorGarraAberta, SensorGarraFechada, SensorPilhaPallet, SensorTDP |
| Colocador de papelão (CP) | MotorPosicaoColeta, MotorPosicaoEntrega, CilindroAvancar, CilindroRecuar, Vacuo | LimitePega, LimiteEntrega, SensorCilindroAv, SensorCilindroRc, SensorPapelaoInferior, SensorPapelaoSuperior, SensorTDP |
| Carro de pallet vazio (CVZ) | LigarDrive, LiberarFreio, Velocidade (Int mm/s), Motor_TDP, Reverso_TDP | PosicaoAtual (Int mm), DriverFDBVelocidade (Int mm/s), DriverEmFalha, SensorExtremoAtivoTDP, Sensor_Pallet_Carro |
| Sensor fotoelétrico | — | Objeto detectado |

Uma entrada sem tag vinculada, ou com o CLP desconectado, usa o valor da caixa **forçar** (aba Propriedades ou **Sinais / I/O**).

## Importar o projeto TIA Portal V18 (.ap18)

O `.ap18` é só um cabeçalho: o projeto fica no banco proprietário do TIA. A leitura é feita pela API oficial **TIA Portal Openness** através do auxiliar .NET `tools/TiaOpennessBridge`.

1. Pré-requisitos: TIA Portal V18 com Openness instalado, usuário no grupo do Windows **Siemens TIA Openness** e .NET SDK.
2. Compile o auxiliar uma vez:
   ```powershell
   tools\TiaOpennessBridge\build.ps1
   ```
3. No app, abra a aba **TIA Portal V18**, clique em **Importar projeto .ap18** e escolha o arquivo.
   - Se o projeto já estiver aberto no TIA, o auxiliar se conecta a essa instância. O TIA pode pedir para você autorizar o acesso Openness.
   - Se não estiver aberto, o auxiliar abre o projeto em um TIA sem interface (cerca de 30 a 60 s), exporta tudo e fecha sem salvar.
   - O TIA não aceita caminhos de pasta de projeto com mais de 143 caracteres.
4. Marque as tabelas de tags e os DBs que devem aparecer na lista de vínculo e vincule as tags aos sinais em **Propriedades → Sinais e tags**.

O auxiliar exporta tabelas de tags (com endereço), DBs e UDTs em SimaticML. O simulador então calcula os offsets dos DBs **não otimizados** seguindo o layout padrão S7: bits empacotados, alinhamento em byte par e Struct/UDT/Array começando e terminando em byte par. O resultado é o endereço absoluto, por exemplo `%DB1001.DBW2`. DBs com **acesso otimizado** aparecem na lista, mas não têm endereço absoluto: desative "Optimized block access" nos DBs que o simulador vai usar.

Também é possível importar arquivos exportados manualmente: tabela de tags (XML/CSV/XLSX), fonte de DB gerada pelo TIA (`.db`/`.scl`/`.udt`, com Struct aninhada e `S7_Optimized_Access`) ou um bloco SimaticML.

## Comunicação com o CLP (aba CLP)

Um projeto pode ter vários CLPs. Na aba **CLP**, use **+ Adicionar CLP** para criar uma conexão por CPU, cada uma com seu IP, rack, slot e ciclo de leitura, e escolha qual **CLP do projeto** ela atende. Ao importar o `.ap18`, cada tabela de tags e cada DB guarda o nome da CPU de onde veio, e as tags vinculadas são lidas e escritas na conexão desse CLP. Para tabelas e DBs importados manualmente, informe o CLP no campo **CLP** da aba TIA Portal V18. Uma conexão marcada "(qualquer / sem CLP)" atende as tags sem CLP definido.


Driver S7 (ISO-on-TCP, porta 102) escrito em Rust puro, em uma thread separada da renderização. Ele lê só as tags vinculadas, agrupando faixas contíguas em uma requisição, e escreve as saídas quando mudam.

Configuração no TIA Portal:
- CPU → Proteção e segurança → Mecanismos de conexão: **Permitir acesso com comunicação PUT/GET**.
- DBs usados pelo simulador: **acesso otimizado desativado**.
- S7-1200/1500: rack 0, slot 1. PLCSIM Advanced: modo de comunicação **TCP/IP** e o IP da instância.
- Para sinais do simulador para o CLP, prefira %M ou DB. Escrever em %I só funciona em endereços sem módulo de entrada atribuído.

## Arquitetura

```text
src/
  main.rs            app Bevy, ordem dos sistemas, cena base
  model.rs           tipos de equipamento, parâmetros, sinais, I/O, vínculos
  visuals.rs         malhas paramétricas (esteira, roletes, M-410iC/185, pallet...)
  editor.rs          câmera, seleção por raio, arrastar/girar/deletar/duplicar, gizmos
  sim.rs             fluxo de caixas, sensores, emissor/removedor, padrão de pallet
  robot.rs           cinemática inversa 4 eixos, programa de paletização, reservas entre robôs
  elevator.rs        elevador/alimentador de pallets e port do FB
  fullcar.rs         carro cheio giratório (CCH) e port dos FBs de movimento e TDP
  pattern.rs         padrões de paletização: gerador de formatos e tela
  io.rs              sinais <-> tags (forçado / CLP)
  communication.rs   thread S7: leitura cíclica agrupada e escrita por mudança
  s7.rs              cliente S7comm (COTP, setup, read/write var) e endereços TIA
  layout.rs          salvar/abrir layout JSON e célula padrão
  ui.rs              painéis egui
  tia_importer/      modelo do projeto, importadores, SimaticML, cálculo de offsets
tools/TiaOpennessBridge/   auxiliar .NET Framework 4.8 (Openness V18)
docs/openness-contract.json  contrato JSON entre o app e o auxiliar
```

## Testes

```powershell
cargo test
# Com um projeto real (o auxiliar precisa estar compilado):
$env:PDT_AP18="C:\curto\projeto.ap18"; cargo test -- --ignored --nocapture openness_end_to_end
```

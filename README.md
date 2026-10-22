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
| Colocador de papelão (CP) | Selgron "MOD COLOCADOR PAPELAO C2370" + FB "CP" | Pórtico com carro entre o depósito de papelão e a entrega (1887 mm), cilindro vertical e cabeça de ventosas. Pode ficar sobre um TDP normal ou sobre uma parada do CVZ |
| Carro de pallet vazio (CVZ) | FB "CVZ" | Carro sobre trilho com TDP que transporta perpendicular ao trilho; o pallet anda com o carro. Inversor (`LigarDrive`, `LiberarFreio`, `Velocidade` em mm/s) e sensor de distância (`PosicaoAtual` em mm) |

As duas mesas terminam na **mesa área de coleta** (FB "Mesa Antecipada Coleta"): motor e batente próprios, sensores Coleta1/Coleta2. Ela recebe o par de fardos, recolhe o batente e dá `Autoriza_Pick`; o robô pega o par inteiro e responde `Robo_Fardos_Coletados`.

Os sinais das mesas têm os **mesmos nomes das variáveis dos FBs** (`Bloqueio_Entrada`, `Sensor_RoletadaPos1`, `GarraAvancar`, `SensorEtapa2Pac3`…), então o CLP real comanda os atuadores e lê os sensores simulados. Sem CLP, a opção **Lógica interna** executa um port dos FBs em Rust (`src/mesa_logic.rs`, mesmos passos e temporizadores) no modo automático, para camadas normais. Os modos última camada ímpar, 10x1 e Fardo10Kg (3/4 fardos por coleta) não estão portados; com o CLP conectado eles funcionam, porque quem executa é o CLP.

As caixas colocadas pelo robô passam a fazer parte do pallet e andam junto com ele pelos TDPs.

## Editor

| Ação | Como fazer |
|---|---|
| Adicionar | Painel **Catálogo**: esteira, Mesa 2 estágios, Mesa pré-forma 3 estágios, TDP, TDP de paletização, robô FANUC M-410iC/185, emissor, magazine de pallets, removedor, sensor fotoelétrico |
| Selecionar | Clique no objeto ou na lista **Cena** |
| Seleção múltipla | `Ctrl`+clique soma ou tira itens; arrastar com o botão esquerdo num espaço vazio desenha um retângulo de seleção (`Ctrl` para somar à seleção atual) |
| Mover | Arraste qualquer item selecionado: o grupo inteiro se move (grade de 0,1 m; segure `Alt` para ignorar a grade). Setas: 0,1 m (`Shift`: 0,5 m). `PgUp`/`PgDn`: altura |
| Rotacionar | `R` / `Shift+R`: ±90° · `Q` / `E`: ±15° (um grupo gira em torno do seu centro) · ou o campo Rotação em Propriedades |
| Deletar | `Del` (todos os selecionados), o botão 🗑 da lista ou **Excluir** |
| Duplicar | `Ctrl+D` (todos os selecionados) |
| Câmera | Botão direito: orbitar · botão do meio ou `Shift`+direito: deslocar · roda: zoom · `F`: focar no selecionado |

Em **Propriedades** você ajusta comprimento, largura, altura e velocidade dos transportadores; no robô, a mesa de coleta, o TDP de paletização e o override de velocidade; no TDP de paletização, o número de camadas, o intertravamento (camadas ímpares giradas 90°) e a lógica interna; no emissor, o intervalo e o tamanho da caixa.

**Arquivo → Salvar/Abrir layout** grava a célula inteira em JSON: equipamentos, vínculos de tags, projeto TIA importado e configurações do CLP.

## Robô FANUC M-410iC/185

O modelo é paramétrico e reproduz a cinemática de paletizador de 4 eixos: J1 é a base, J2/J3 formam o braço com paralelogramo que mantém o flange horizontal e J4 gira o punho. O alcance é de aproximadamente 3,1 m, como na folha de dados. O programa interno faz o papel da rotina de paletização do controlador: espera a caixa no fim do transportador, coleta, sobe até a altura segura, gira, deposita na posição do padrão, sobe de novo e repete. Quando o pallet fica completo, sinaliza **Pallet completo**.

O visual usa o **modelo do fabricante** (STEP SolidWorks M-410iC185, com pedestal), convertido para 12 peças em `assets/models/fanuc_m410ic185/*.glb`. Cada peça acompanha a sua articulação: base e pedestal fixos, carrossel J1, braço J2 (S302), alavanca e biela do J3 (S303/S304), braço J3 (S402), paralelogramo de nivelamento (S305/S401/S403), punho (S501) e flange J4 (S502). As medidas da cinemática foram tiradas do próprio modelo: J2 a 1,110 m do chão, offset 0,394 m, braços de 1,220 m e 1,306 m, e flange 0,244 m à frente e 0,159 m abaixo do pino do punho. Isso dá um alcance horizontal até o flange de 3,164 m (catálogo: 3143 mm). Se os arquivos não estiverem em `assets/`, o simulador usa o modelo paramétrico. Ao rodar o executável fora do `cargo run`, copie a pasta `assets` para junto do `.exe` ou defina `BEVY_ASSET_ROOT`.

### Cabeçote

O robô usa o cabeçote Selgron "MOD CAB 4 FARDOS" (0323P01-010000): quadro de 1234 × 500 mm com acoplamento 435 mm acima da base (o TCP fica na base do quadro, onde o topo dos fardos encosta), divisória central de 229 mm e duas pinças laterais de 295 mm acionadas pelos cilindros CIL 40-420. As pinças fecham sobre o grupo coletado e abrem ao depositar.

## Colocador de papelão (CP)

O carro do CP corre entre a posição de **pega**, sobre o depósito com a pilha de papelões, e a de **entrega** (círculo amarelo quando o CP está selecionado), onde o centro do pallet deve parar. A cabeça de ventosas é flutuante: o `SensorPapelaoInferior` fica ligado com a cabeça solta e sem papelão, e o `SensorPapelaoSuperior` liga quando ela é pressionada contra a pilha ou o pallet. A folha aplicada fica sobre o pallet, anda com ele, e o robô empilha em cima dela.

- **Sobre um TDP normal:** um pallet sem papelão que chega sob a entrega é segurado ali (como o TDP parando no sensor) até o fim do ciclo, e a fila atrás dele espera.
- **Como parada do CVZ:** marque a caixa **CP** numa das paradas do carro. Depois de receber o pallet na coleta, o carro vai primeiro até essa parada, espera o papelão e só então segue para o destino.

Sem CLP, a **lógica interna** porta os passos automáticos do FB CP (0–2, 100–117, 200/201, 400): desce na pilha, liga o vácuo ao perder os dois sensores, para no sensor superior, estabiliza, sobe, confere o sensor inferior (sem papelão → passo 400), vai à entrega, desce até o sensor superior, desliga o vácuo, sobe e volta. A opção **Aplicar 2x papelão** repete a aplicação. O botão **Reabastecer depósito** repõe a pilha. A escolha "aplicar ou não" por ID de linha do FB vem do CLP; internamente vale a opção **Aplicar papelão**.

## Carro de pallet vazio (CVZ)

O trilho fica ao longo do X local do equipamento e o TDP do carro transporta ao longo do Z local: recebe de um lado do trilho e entrega do outro. O sensor de distância fica no início do trilho e mede o centro do carro em mm. As paradas (coleta, posição A, B, …) são configuradas em Propriedades.

Sem CLP, a **lógica interna** porta o posicionamento do FB CVZ (passo 201): velocidade nominal, redução proporcional a partir de 500 mm, velocidade de precisão nos últimos 50 mm, desliga o inversor na tolerância de 5 mm e trava o freio quando a velocidade real chega a zero. O ciclo interno é simples: recebe o pallet na coleta, leva às posições A, B… em sequência, descarrega e volta. A escolha do destino do FB real depende dos IDs de rastreamento dos pallets; com o CLP conectado, ela vem do CLP.

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
  robot.rs           cinemática inversa 4 eixos e programa de paletização
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

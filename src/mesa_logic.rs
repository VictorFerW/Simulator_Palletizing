//! Port of the Selgron mesa FBs (SCL) used when the cell runs without a PLC:
//! "Mesa Antecipada Roletada" (2 estágios), "Mesa 3 Estágios" (roletada +
//! pré-formadora) and "Mesa Antecipada Coleta" (common to both).
//!
//! Step numbers, timers and flags keep the names of the original code so the
//! behaviour can be compared side by side. Only the automatic mode is ported.
//! Última camada ímpar (mesa 2 estágios): when the robot asks for a single fardo
//! (`single_request`, the last product of a pallet) only one enters, the empurrador
//! pushes it and the roletada releases it to the coleta, which then waits for the
//! pushed side only. 10x1 and 10 kg variants are off (`Produto_10x1 = 0`, `Fardo10Kg = 0`).
//! Actuators driven by a connected PLC always win over this logic.

use bevy::prelude::*;

use crate::model::{Equipment, EquipmentKind, Io, SimControl};

/// IEC TON: Q after IN has been true for PT seconds.
#[derive(Default, Clone, Copy, Debug)]
struct Ton {
    et: f32,
}

impl Ton {
    fn run(&mut self, input: bool, pt: f32, dt: f32) -> bool {
        if input {
            self.et = (self.et + dt).min(pt + 1.0);
        } else {
            self.et = 0.0;
        }
        input && self.et >= pt
    }
}

// Timer presets (instance DB values of a typical installation).
const TEMPO_BLOQUEIO_ENTRADA_FARDOS: f32 = 0.8;
const TEMPO_ENTRADA_ETAPA2: f32 = 0.3;
const TEMPO_SIMULTANEIDADE: f32 = 1.5;
const TEMPO_CILINDRO_COLETA: f32 = 0.5;
const TEMPO_RECEBIMENTO_FARDOS: f32 = 0.5;
const TEMPO_RETIRADA_FARDOS: f32 = 1.0;
const TEMPO_LIBERAR_BATENTE: f32 = 0.5;
const TEMPO_ATRASO_BLOQUEIO_ENTRADA: f32 = 0.5;
const TEMPO_ATRASO_BLOQUEIO_MEIO: f32 = 0.5;
const TEMPO_ENTRADA_FARDOS: f32 = 0.4;

/// FB "Mesa Antecipada Coleta".
#[derive(Default, Debug)]
pub struct Coleta {
    pub passo: u32,
    motor: bool,
    batente: bool,
    pub pronto_receber: bool,
    pub recebido: bool,
    pub autoriza_pick: bool,
    pronto_coleta: bool,
    trig_recebimento: bool,
    trig_retirada: bool,
    trig_liberar: bool,
    t_recebimento: Ton,
    t_retirada: Ton,
    t_liberar: Ton,
    /// A single fardo (pushed side) is on its way / on the coleta.
    pub single: bool,
}

impl Coleta {
    fn tick(&mut self, c1: bool, c2: bool, roletada_transferindo: bool, robo_coletados: bool, dt: f32) {
        let q_rec = self.t_recebimento.run(self.trig_recebimento, TEMPO_RECEBIMENTO_FARDOS, dt);
        let q_ret = self.t_retirada.run(self.trig_retirada, TEMPO_RETIRADA_FARDOS, dt);
        let q_lib = self.t_liberar.run(self.trig_liberar, TEMPO_LIBERAR_BATENTE, dt);
        match self.passo {
            0 => {
                // Passo de reset
                self.motor = false;
                self.batente = true;
                self.pronto_receber = false;
                self.recebido = false;
                self.autoriza_pick = false;
                self.pronto_coleta = false;
                self.trig_liberar = false;
                self.trig_recebimento = false;
                self.trig_retirada = false;
                self.passo = if !c2 || !c1 { 1 } else { 3 };
            }
            1 => {
                // Pronto receber
                self.pronto_receber = true;
                self.batente = true;
                if roletada_transferindo {
                    self.motor = true;
                    self.passo = 2;
                }
            }
            2 => {
                // Recebido
                self.motor = true;
                self.pronto_receber = true;
                if (c1 || self.single) && c2 {
                    self.trig_recebimento = true;
                }
                if q_rec {
                    self.recebido = true;
                    self.pronto_receber = false;
                    self.passo = 3;
                    self.motor = false;
                    self.trig_recebimento = false;
                }
            }
            3 => {
                // Recolhe batente e libera coleta
                self.motor = false;
                self.pronto_coleta = true;
                if (c1 || self.single) && c2 {
                    self.trig_liberar = true;
                }
                if q_lib {
                    self.recebido = false;
                    self.batente = false;
                    self.pronto_receber = false;
                    self.passo = 4;
                    self.autoriza_pick = true;
                    self.trig_liberar = false;
                }
            }
            4 => {
                // Verifica os fardos coletados e reseta ciclo
                self.motor = false;
                self.batente = false;
                self.autoriza_pick = true;
                if (!c2 || !c1) && robo_coletados {
                    self.trig_retirada = true;
                }
                if q_ret {
                    self.batente = true;
                    self.single = false;
                    self.passo = 1;
                    self.pronto_coleta = false;
                    self.autoriza_pick = false;
                    self.trig_retirada = false;
                }
            }
            _ => self.passo = 0,
        }
    }
}

/// FB "Mesa Antecipada Roletada para fardos" (mesa 2 estágios).
#[derive(Default, Debug)]
pub struct Roletada2 {
    pub passo_entrada: u32,
    pub passo_roletada: u32,
    motor_entrada: bool,
    motor_roletada: bool,
    motor: bool,
    bloqueio_entrada: bool,
    bloqueio_coleta: bool,
    empurrador_av: bool,
    empurrador_rc: bool,
    entrada_transferindo: bool,
    roletada_pronto_receber: bool,
    roletada_recebida: bool,
    pub antecipada_transferindo: bool,
    flag_preparo_especial: bool,
    trig_entrada_fardos: bool,
    trig_entrada_etapa2: bool,
    trig_simultaneidade: bool,
    trig_cilindro_coleta: bool,
    t_entrada_fardos: Ton,
    t_entrada_etapa2: Ton,
    t_simultaneidade: Ton,
    t_cilindro_coleta: Ton,
    /// Ultima_Camada_Impar: the next group is a single fardo.
    ultima_impar: bool,
    /// The fardo being transferred to the coleta is a single one.
    pub single_sent: bool,
}

struct Sensors2 {
    esteira_fim: bool,
    antes_bloqueio: bool,
    apos_bloqueio: bool,
    pos1: bool,
    pos2: bool,
    pos3: bool,
    emp_av: bool,
    emp_rc: bool,
    coleta1: bool,
    coleta2: bool,
}

impl Roletada2 {
    fn tick(&mut self, s: &Sensors2, coleta_pronto_receber: bool, coleta_recebido: bool, dt: f32) {
        let q_entrada_fardos = self.t_entrada_fardos.run(self.trig_entrada_fardos, TEMPO_BLOQUEIO_ENTRADA_FARDOS, dt);
        let q_etapa2 = self.t_entrada_etapa2.run(self.trig_entrada_etapa2, TEMPO_ENTRADA_ETAPA2, dt);
        let q_simult = self.t_simultaneidade.run(self.trig_simultaneidade, TEMPO_SIMULTANEIDADE, dt);
        let q_cilindro = self.t_cilindro_coleta.run(self.trig_cilindro_coleta, TEMPO_CILINDRO_COLETA, dt);

        // Passo Entrada
        match self.passo_entrada {
            0 => {
                self.motor_entrada = s.antes_bloqueio || s.esteira_fim;
                self.bloqueio_entrada = true;
                self.trig_entrada_fardos = false;
                self.passo_entrada = 1;
                self.entrada_transferindo = false;
            }
            1 => {
                // Liga a esteira quando vai entrar um produto na mesa
                self.motor_entrada = false;
                if s.antes_bloqueio || s.esteira_fim {
                    self.motor_entrada = true;
                    if self.roletada_pronto_receber {
                        self.bloqueio_entrada = false;
                        self.entrada_transferindo = true;
                    } else {
                        self.bloqueio_entrada = true;
                    }
                    self.passo_entrada = 2;
                }
            }
            2 => {
                // Verifica a mesa roletada para saber se pode mandar o produto
                self.motor_entrada = true;
                self.entrada_transferindo = true;
                if self.roletada_pronto_receber {
                    self.bloqueio_entrada = false;
                    if s.apos_bloqueio || (s.pos1 && !q_simult) {
                        self.passo_entrada = 3;
                        self.trig_entrada_fardos = true;
                    }
                } else {
                    self.motor_entrada = !s.antes_bloqueio;
                    self.bloqueio_entrada = true;
                }
            }
            3 => {
                // Levanta o bloqueio e verifica se foi recebido o produto
                self.entrada_transferindo = true;
                self.motor_entrada = true;
                self.trig_entrada_fardos = true;
                if q_entrada_fardos {
                    self.bloqueio_entrada = true;
                }
                if self.roletada_recebida {
                    self.bloqueio_entrada = true;
                    self.trig_entrada_fardos = false;
                    self.passo_entrada = 1;
                    self.entrada_transferindo = false;
                }
            }
            _ => self.passo_entrada = 0,
        }

        // Passo Roletada
        let push = |me: &mut Self| {
            me.empurrador_av = true;
            me.empurrador_rc = false;
            me.motor_roletada = false;
            me.roletada_recebida = true;
            if (s.pos2 || s.pos3) && s.emp_av {
                me.roletada_recebida = false;
                me.empurrador_av = false;
                me.empurrador_rc = true;
                me.passo_roletada = 4;
            }
        };
        match self.passo_roletada {
            0 => {
                self.empurrador_av = false;
                self.empurrador_rc = true;
                self.bloqueio_coleta = true;
                self.motor_roletada = false;
                self.roletada_pronto_receber = false;
                self.roletada_recebida = false;
                if s.emp_rc {
                    self.passo_roletada = if !s.pos1 && !s.pos2 && !s.pos3 {
                        1 // Mesa vazia
                    } else if s.pos1 && !s.pos2 && !s.pos3 {
                        3 // Um produto na mesa - empurra
                    } else {
                        4
                    };
                }
            }
            10 if self.ultima_impar && !self.entrada_transferindo && !s.pos1 && (s.pos2 || s.pos3) => {
                // Última camada ímpar: no second fardo, release the pushed one.
                self.passo_roletada = 4;
            }
            1 | 10 | 21 => {
                // Recebe fardo (1: primeiro, 10: segundo, 21: última camada)
                self.empurrador_av = false;
                self.empurrador_rc = true;
                self.bloqueio_coleta = true;
                self.roletada_pronto_receber = s.emp_rc;
                if self.entrada_transferindo && s.emp_rc {
                    self.motor_roletada = true;
                    self.passo_roletada = match self.passo_roletada {
                        1 => 2,
                        10 => 11,
                        _ => 22,
                    };
                }
            }
            2 => {
                // Primeiro fardo - recebido
                self.empurrador_av = false;
                self.empurrador_rc = true;
                self.motor_roletada = true;
                self.roletada_pronto_receber = true;
                self.bloqueio_coleta = true;
                if s.pos1 {
                    self.trig_entrada_etapa2 = true;
                }
                if (q_etapa2 || TEMPO_ENTRADA_ETAPA2 == 0.0) && self.trig_entrada_etapa2 {
                    self.motor_roletada = false;
                    self.roletada_recebida = true;
                    self.roletada_pronto_receber = false;
                    self.passo_roletada = 3;
                    self.trig_entrada_etapa2 = false;
                }
            }
            3 | 5 => push(self), // Empurra (região 10x1, Ultima_Camada_Impar = 0)
            4 => {
                // Verifica - última camada ímpar / 10x1
                self.motor_roletada = false;
                self.empurrador_av = false;
                self.empurrador_rc = true;
                if s.emp_rc {
                    if self.ultima_impar && !s.pos1 && (s.pos2 || s.pos3) {
                        self.passo_roletada = 50; // Última camada ímpar: só o fardo empurrado
                    } else if s.coleta1 && !s.coleta2 {
                        self.passo_roletada = 20;
                    } else if !s.pos1 {
                        self.passo_roletada = 10; // Modo padrão
                    } else if s.pos1 && s.pos2 {
                        self.passo_roletada = 50;
                    }
                }
            }
            11 => {
                // Segundo fardo - recebido
                self.empurrador_av = false;
                self.empurrador_rc = true;
                self.motor_roletada = true;
                self.roletada_pronto_receber = true;
                self.bloqueio_coleta = true;
                if s.pos1 {
                    self.trig_entrada_etapa2 = true;
                }
                if (q_etapa2 || TEMPO_ENTRADA_ETAPA2 == 0.0) && s.pos1 && s.emp_rc {
                    self.motor_roletada = false;
                    self.roletada_recebida = true;
                    self.roletada_pronto_receber = false;
                    self.passo_roletada = 50;
                    self.trig_entrada_etapa2 = false;
                }
            }
            20 => {
                // Última camada - toma decisão
                self.empurrador_av = false;
                self.empurrador_rc = true;
                let pushed = s.pos2 || s.pos3;
                if (pushed && s.coleta1 && coleta_pronto_receber)
                    || (s.pos1 && pushed && coleta_pronto_receber && !s.coleta1 && !s.coleta2)
                {
                    self.passo_roletada = 50;
                } else if !s.pos1 && pushed && self.flag_preparo_especial && !s.coleta1 && coleta_pronto_receber {
                    self.passo_roletada = 21;
                } else if !s.pos1 && pushed && coleta_pronto_receber && !s.coleta1 && !s.coleta2 {
                    self.passo_roletada = 4;
                }
            }
            22 => {
                // Última camada - segundo fardo - recebido
                self.empurrador_av = false;
                self.empurrador_rc = true;
                self.motor_roletada = true;
                self.roletada_pronto_receber = true;
                self.bloqueio_coleta = true;
                if s.pos1 {
                    self.motor_roletada = false;
                    self.roletada_recebida = true;
                    self.roletada_pronto_receber = false;
                    self.passo_roletada = 50;
                }
            }
            50 => {
                // Transferência - verifica sinal
                self.empurrador_av = false;
                self.empurrador_rc = true;
                self.bloqueio_coleta = true;
                self.roletada_recebida = false;
                self.roletada_pronto_receber = false;
                if coleta_pronto_receber {
                    self.antecipada_transferindo = true;
                    self.motor_roletada = true;
                    self.bloqueio_coleta = false;
                    self.single_sent = self.ultima_impar && !s.pos1;
                    self.passo_roletada = 51;
                    self.flag_preparo_especial = false;
                }
            }
            51 => {
                // Transferência - verifica simultaneidade/bloqueio
                self.trig_simultaneidade = true;
                self.motor_roletada = true;
                self.trig_cilindro_coleta = (s.coleta1 || self.single_sent) && s.coleta2;
                if q_cilindro {
                    self.bloqueio_coleta = true;
                }
                if q_simult && s.emp_rc {
                    self.roletada_pronto_receber = true;
                }
                if coleta_recebido {
                    if q_simult {
                        if self.entrada_transferindo && s.pos1 {
                            self.roletada_recebida = true;
                            self.roletada_pronto_receber = false;
                            self.passo_roletada = 3;
                        }
                    } else {
                        self.passo_roletada = 1;
                    }
                    self.trig_cilindro_coleta = false;
                    self.trig_simultaneidade = false;
                    self.bloqueio_coleta = true;
                    self.motor_roletada = false;
                    self.antecipada_transferindo = false;
                }
            }
            _ => self.passo_roletada = 0,
        }

        // Espelhamento motor
        if (self.motor_entrada || self.motor_roletada) && !self.empurrador_av {
            self.motor = true;
        }
        if (!self.motor_entrada && !self.motor_roletada) || self.empurrador_av {
            self.motor = false;
        }
    }
}

/// FB "Mesa 3 Estágios" (roletada com garra + pré-formadora com empurrador).
#[derive(Default, Debug)]
pub struct PreForma3 {
    pub passo_roletada: u32,
    pub passo_preformadora: u32,
    motor_roletada: bool,
    motor_preformadora: bool,
    bloqueio_entrada: bool,
    bloqueio_meio: bool,
    bloqueio_coleta: bool,
    garra_sobe: bool,
    garra_desce: bool,
    garra_avancar: bool,
    garra_recuar: bool,
    empurrador_avancar: bool,
    empurrador_recuar: bool,
    solicitar_fardo: bool,
    pub antecipada_transferindo: bool,
    espera_pac_rol: bool,
    espera_ent_pac_rol: bool,
    espera_pac_pre: bool,
    espera_ent_pac_pre: bool,
    espera_simul_rol_pre: bool,
    espera_simul_pre_col: bool,
    t_sep_rol: Ton,
    t_ent_pac_rol: Ton,
    t_sep_pre: Ton,
    t_ent_pre: Ton,
    t_simu_rol_pre: Ton,
    t_simul_pre_col: Ton,
}

struct Sensors3 {
    esteira: bool,
    pac1_prox_bloq: bool,
    e1p1: bool,
    e1p2: bool,
    e1p3: bool,
    garra_levantada: bool,
    garra_recolhida: bool,
    garra_av: bool,
    garra_rec: bool,
    e2p1: bool,
    e2p2: bool,
    e2p3: bool,
    emp_rec: bool,
}

impl PreForma3 {
    fn tick(&mut self, s: &Sensors3, coleta_pronta_receber: bool, coleta_recebido: bool, dt: f32) {
        let q_sep_rol = self.t_sep_rol.run(self.espera_pac_rol, TEMPO_ATRASO_BLOQUEIO_ENTRADA, dt);
        let q_ent_pac_rol = self.t_ent_pac_rol.run(self.espera_ent_pac_rol, TEMPO_ENTRADA_FARDOS, dt);
        let q_sep_pre = self.t_sep_pre.run(self.espera_pac_pre, TEMPO_ATRASO_BLOQUEIO_MEIO, dt);
        let q_ent_pre = self.t_ent_pre.run(self.espera_ent_pac_pre, TEMPO_ENTRADA_FARDOS, dt);
        let q_simu_rol_pre = self.t_simu_rol_pre.run(self.espera_simul_rol_pre, TEMPO_SIMULTANEIDADE, dt);
        let q_simul_pre_col = self.t_simul_pre_col.run(self.espera_simul_pre_col, TEMPO_SIMULTANEIDADE, dt);

        // Condição geral: garra recuada pode subir.
        let raise_if_back = |me: &mut Self| {
            if s.garra_rec {
                me.garra_sobe = true;
                me.garra_desce = false;
            }
        };

        match self.passo_roletada {
            0 => {
                // Vetor de reset (Fardo10Kg = 0)
                self.garra_avancar = false;
                self.garra_desce = true;
                self.garra_recuar = true;
                self.garra_sobe = false;
                self.bloqueio_meio = true;
                self.bloqueio_entrada = true;
                if !s.e1p1 && !s.e1p2 && !s.e1p3 {
                    self.passo_roletada = 1;
                }
                if s.e1p1 && !s.e1p2 && !s.e1p3 {
                    self.passo_roletada = 2;
                }
                if !s.e1p1 && (s.e1p2 || s.e1p3) {
                    self.passo_roletada = 3;
                }
                if s.e1p1 && s.e1p2 {
                    self.passo_roletada = 6;
                }
            }
            1 => {
                // Sem fardos na mesa roletada entrada
                if s.esteira || s.pac1_prox_bloq {
                    self.motor_roletada = true;
                    self.bloqueio_entrada = false;
                    self.passo_roletada = 2;
                    self.espera_ent_pac_rol = false;
                }
                raise_if_back(self);
            }
            2 => {
                // Aguarda fardo acionar sensor na roleta
                raise_if_back(self);
                self.espera_pac_rol = s.pac1_prox_bloq || s.e1p1;
                self.espera_ent_pac_rol = s.e1p1 && self.bloqueio_entrada;
                if q_sep_rol {
                    self.bloqueio_entrada = true;
                }
                if q_ent_pac_rol {
                    self.motor_roletada = false;
                    self.espera_pac_rol = false;
                    self.espera_ent_pac_rol = false;
                    self.passo_roletada = 3;
                }
            }
            3 => {
                // Aguardar fardos chegarem na posição e inicia movimento da garra
                raise_if_back(self);
                if s.garra_levantada {
                    self.garra_avancar = true;
                    self.garra_recuar = false;
                    self.espera_ent_pac_rol = false;
                    self.passo_roletada = 4;
                }
            }
            4 => {
                // Aguarda a garra avançar e acionar o sensor do pacote 2
                if s.garra_av && s.e1p2 {
                    self.garra_desce = true;
                    self.garra_sobe = false;
                    self.garra_recuar = true;
                    self.garra_avancar = false;
                }
                if s.garra_recolhida {
                    self.passo_roletada = 5;
                }
            }
            5 => {
                // Já tem um fardo na posição 2: recebe o próximo
                self.bloqueio_entrada = false;
                if s.esteira {
                    self.motor_roletada = true;
                }
                self.espera_pac_rol = s.pac1_prox_bloq || s.e1p1;
                if s.e1p1 {
                    self.espera_ent_pac_rol = true;
                }
                if q_sep_rol {
                    self.espera_pac_rol = false;
                    self.bloqueio_entrada = true;
                    self.passo_roletada = 6;
                }
                raise_if_back(self);
            }
            6 => {
                // Aguarda pré-formadora estar recebendo fardos
                if s.e1p1 {
                    self.espera_ent_pac_rol = true;
                }
                if q_ent_pac_rol {
                    self.motor_roletada = false;
                }
                if self.solicitar_fardo && !s.e2p1 && !s.e2p3 {
                    self.motor_preformadora = true;
                    self.espera_ent_pac_rol = false;
                    self.bloqueio_meio = false;
                    self.passo_roletada = 7;
                    self.motor_roletada = true;
                }
                raise_if_back(self);
            }
            7 => {
                // Verifica se os fardos chegaram para subir o bloqueio
                if s.e2p1 && s.e2p3 {
                    self.espera_pac_pre = true;
                }
                if q_sep_pre {
                    self.bloqueio_meio = true;
                    if self.bloqueio_entrada && !q_sep_rol {
                        self.motor_roletada = false;
                    }
                }
                // Simultaneidade
                if !s.e1p1 && !s.e1p2 && !s.e1p3 && !s.pac1_prox_bloq && s.esteira {
                    self.espera_simul_rol_pre = true;
                }
                if q_simu_rol_pre && s.esteira {
                    self.bloqueio_entrada = false;
                    self.motor_roletada = true;
                }
                self.espera_pac_rol = (s.pac1_prox_bloq || s.e1p1) && self.espera_simul_rol_pre;
                if s.e1p1 && self.espera_simul_rol_pre {
                    self.espera_ent_pac_rol = true;
                }
                if q_sep_rol {
                    self.bloqueio_entrada = true;
                }
                if q_simu_rol_pre && q_sep_rol && q_sep_pre {
                    self.espera_simul_rol_pre = false;
                    self.espera_ent_pac_rol = false;
                    self.espera_pac_pre = false;
                    self.passo_roletada = 2;
                }
                if self.passo_roletada == 7 && self.bloqueio_meio && !q_simu_rol_pre {
                    self.espera_simul_rol_pre = false;
                    self.espera_ent_pac_rol = false;
                    self.espera_pac_pre = false;
                    self.motor_roletada = false;
                    self.passo_roletada = 1;
                }
            }
            _ => self.passo_roletada = 0,
        }

        match self.passo_preformadora {
            0 => {
                self.empurrador_avancar = false;
                self.empurrador_recuar = true;
                self.bloqueio_coleta = true;
                if !s.e2p1 && !s.e2p3 {
                    self.passo_preformadora = 1;
                }
                // Um ou dois fardos já na pré: segue para a transferência.
                if s.e2p1 || s.e2p3 {
                    self.passo_preformadora = 3;
                }
            }
            1 => {
                // Início do ciclo
                self.bloqueio_coleta = true;
                self.espera_simul_pre_col = false;
                if s.emp_rec {
                    self.solicitar_fardo = true;
                    if !self.bloqueio_meio && self.solicitar_fardo {
                        self.passo_preformadora = 2;
                        self.solicitar_fardo = false;
                    }
                }
            }
            2 => {
                // Aguardar o recebimento dos fardos
                if s.e2p1 && s.e2p3 {
                    self.espera_ent_pac_pre = true;
                }
                if q_ent_pre {
                    self.motor_preformadora = false;
                    self.espera_ent_pac_pre = false;
                    self.passo_preformadora = 3;
                }
            }
            3 => {
                // Verifica se a coleta está recebendo fardos
                if coleta_pronta_receber {
                    self.passo_preformadora = 4;
                    self.bloqueio_coleta = false;
                    self.antecipada_transferindo = true;
                    self.motor_preformadora = true;
                }
            }
            4 => {
                // Aguarda receber sinal que foi recebido
                if coleta_recebido && !q_simul_pre_col {
                    self.antecipada_transferindo = false;
                    self.bloqueio_coleta = true;
                    self.motor_preformadora = false;
                    self.passo_preformadora = 1;
                    self.espera_simul_pre_col = false;
                }
                if !s.e2p1 && !s.e2p2 && !s.e2p3 {
                    self.espera_simul_pre_col = true;
                }
                if coleta_recebido && q_simul_pre_col {
                    self.antecipada_transferindo = false;
                    self.bloqueio_coleta = true;
                }
                if s.emp_rec && q_simul_pre_col {
                    self.solicitar_fardo = true;
                    if !self.bloqueio_meio && self.solicitar_fardo {
                        self.passo_preformadora = 20;
                        self.espera_simul_pre_col = false;
                        self.solicitar_fardo = false;
                    }
                }
                if self.bloqueio_coleta && self.bloqueio_meio && q_simul_pre_col && self.passo_preformadora == 4 {
                    self.passo_preformadora = 1;
                    self.espera_simul_pre_col = false;
                    self.motor_preformadora = false;
                }
            }
            20 => {
                // Simultaneidade - quando vai receber 2 fardos
                self.motor_preformadora = true;
                if coleta_recebido {
                    self.antecipada_transferindo = false;
                    self.bloqueio_coleta = true;
                }
                if (s.e2p1 || s.e2p2) && s.e2p3 {
                    self.espera_ent_pac_pre = true;
                }
                if q_ent_pre {
                    self.motor_preformadora = false;
                }
                if self.bloqueio_coleta && q_ent_pre {
                    self.espera_ent_pac_pre = false;
                    self.passo_preformadora = 3;
                }
            }
            _ => self.passo_preformadora = 0,
        }
    }

    fn motor(&self) -> bool {
        self.motor_preformadora || self.motor_roletada
    }
}

#[derive(Debug)]
pub enum MesaProgram {
    Antecipada(Roletada2),
    PreForma(PreForma3),
}

/// Internal controller of a mesa: roletada FB + coleta FB, plus the robot handshake.
#[derive(Component, Debug)]
pub struct MesaLogic {
    pub program: MesaProgram,
    pub coleta: Coleta,
    /// Set by the robot after lifting the fardos (Robo_Fardos_Coletados).
    pub robot_collected: bool,
    /// Set by the robot when the pallet needs one last product (última camada ímpar).
    pub single_request: bool,
}

impl MesaLogic {
    pub fn for_kind(kind: EquipmentKind) -> Option<Self> {
        let program = match kind {
            EquipmentKind::Table2Stage => MesaProgram::Antecipada(Roletada2::default()),
            EquipmentKind::Table3Stage => MesaProgram::PreForma(PreForma3::default()),
            _ => return None,
        };
        Some(Self { program, coleta: Coleta::default(), robot_collected: false, single_request: false })
    }

    pub fn status(&self) -> String {
        match &self.program {
            MesaProgram::Antecipada(r) => format!(
                "Passo entrada {} · roletada {} · coleta {}",
                r.passo_entrada, r.passo_roletada, self.coleta.passo
            ),
            MesaProgram::PreForma(p) => format!(
                "Passo roletada {} · pré-formadora {} · coleta {}",
                p.passo_roletada, p.passo_preformadora, self.coleta.passo
            ),
        }
    }
}

/// Writes an actuator unless the PLC is driving it.
fn drive(io: &mut Io, key: &'static str, value: bool) {
    if !io.from_plc.contains(key) {
        io.set_bool(key, value);
    }
}

pub fn run_mesas(time: Res<Time>, control: Res<SimControl>, mut mesas: Query<(&Equipment, &mut Io, &mut MesaLogic)>) {
    if !control.running {
        return;
    }
    let dt = time.delta_secs() * control.time_scale;
    for (eq, mut io, mut logic) in &mut mesas {
        if !eq.params.auto_logic {
            continue;
        }
        let logic = &mut *logic;
        let (c1, c2) = (io.get_bool("Sensor_Coleta1"), io.get_bool("Sensor_Coleta2"));
        let (pronto, recebido) = (logic.coleta.pronto_receber, logic.coleta.recebido);
        let transferindo = match &mut logic.program {
            MesaProgram::Antecipada(r) => {
                let s = Sensors2 {
                    esteira_fim: io.get_bool("Sensor_EsteiraFim"),
                    antes_bloqueio: io.get_bool("Sensor_AntesBloqueio"),
                    apos_bloqueio: io.get_bool("Sensor_AposBloqueio"),
                    pos1: io.get_bool("Sensor_RoletadaPos1"),
                    pos2: io.get_bool("Sensor_RoletadaPos2"),
                    pos3: io.get_bool("Sensor_RoletadaPos3"),
                    emp_av: io.get_bool("Sensor_EmpurradorAv"),
                    emp_rc: io.get_bool("Sensor_EmpurradorRc"),
                    coleta1: c1,
                    coleta2: c2,
                };
                r.ultima_impar = logic.single_request;
                r.tick(&s, pronto, recebido, dt);
                if r.single_sent {
                    logic.single_request = false;
                    logic.coleta.single = true;
                }
                drive(&mut io, "Motor", r.motor);
                drive(&mut io, "Bloqueio_Entrada", r.bloqueio_entrada);
                drive(&mut io, "Bloqueio_Coleta", r.bloqueio_coleta);
                drive(&mut io, "EmpurradorAv", r.empurrador_av);
                drive(&mut io, "EmpurradorRc", r.empurrador_rc);
                r.antecipada_transferindo
            }
            MesaProgram::PreForma(p) => {
                let s = Sensors3 {
                    esteira: io.get_bool("SensorEsteira"),
                    pac1_prox_bloq: io.get_bool("SensorEtapa1Pac1ProxBloq"),
                    e1p1: io.get_bool("SensorEtapa1Pac1"),
                    e1p2: io.get_bool("SensorEtapa1Pac2"),
                    e1p3: io.get_bool("SensorEtapa1Pac3"),
                    garra_levantada: io.get_bool("SensorGarraLevantada"),
                    garra_recolhida: io.get_bool("SensorGarraRecolhida"),
                    garra_av: io.get_bool("SensorGarraAv"),
                    garra_rec: io.get_bool("SensorGarraRec"),
                    e2p1: io.get_bool("SensorEtapa2Pac1"),
                    e2p2: io.get_bool("SensorEtapa2Pac2"),
                    e2p3: io.get_bool("SensorEtapa2Pac3"),
                    emp_rec: io.get_bool("SensorEmpurradorRec"),
                };
                p.tick(&s, pronto, recebido, dt);
                drive(&mut io, "Motor", p.motor());
                drive(&mut io, "BloqueioEntrada", p.bloqueio_entrada);
                drive(&mut io, "BloqueioMeio", p.bloqueio_meio);
                drive(&mut io, "BloqueioColeta", p.bloqueio_coleta);
                drive(&mut io, "GarraSobe", p.garra_sobe);
                drive(&mut io, "GarraDesce", p.garra_desce);
                drive(&mut io, "GarraAvancar", p.garra_avancar);
                drive(&mut io, "GarraRecuar", p.garra_recuar);
                drive(&mut io, "EmpurradorAvancar", p.empurrador_avancar);
                drive(&mut io, "EmpurradorRecuar", p.empurrador_recuar);
                p.antecipada_transferindo
            }
        };
        if std::env::var("PDT_TRACE").is_ok() {
            let on: Vec<&str> = eq.kind.signals().iter().filter(|s| io.get_bool(s.key)).map(|s| s.key).collect();
            eprintln!("[{:.2}] {} {} | {}", time.elapsed_secs(), eq.name, logic.status(), on.join(" "));
        }
        let was_waiting_pick = logic.coleta.passo == 4;
        logic.coleta.tick(c1, c2, transferindo, logic.robot_collected, dt);
        if was_waiting_pick && logic.coleta.passo != 4 {
            logic.robot_collected = false;
        }
        drive(&mut io, "Motor_Coleta", logic.coleta.motor);
        drive(&mut io, "Batente_Coleta", logic.coleta.batente);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coleta_handshake() {
        let mut c = Coleta::default();
        let dt = 0.1;
        c.tick(false, false, false, false, dt); // reset -> 1
        assert_eq!(c.passo, 1);
        c.tick(false, false, false, false, dt); // passo 1 executa no scan seguinte
        assert!(c.pronto_receber);
        c.tick(false, false, true, false, dt); // roletada transferindo
        assert_eq!(c.passo, 2);
        assert!(c.motor);
        for _ in 0..10 {
            c.tick(true, true, true, false, dt);
        }
        assert!(c.passo >= 3);
        for _ in 0..10 {
            c.tick(true, true, false, false, dt);
        }
        assert_eq!(c.passo, 4);
        assert!(c.autoriza_pick && !c.batente);
        for _ in 0..15 {
            c.tick(false, false, false, true, dt);
        }
        assert_eq!(c.passo, 1);
        assert!(c.batente && !c.autoriza_pick);
    }
}

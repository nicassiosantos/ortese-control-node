#!/usr/bin/env python3
"""
plotar_telemetria.py

Gera graficos a partir do CSV gravado pelo logger_node.
Util para analisar o comportamento do controle e gerar figuras do TCC.

Uso:
    python3 plotar_telemetria.py /tmp/control_log.csv
    python3 plotar_telemetria.py /tmp/control_log.csv --salvar figuras/

Colunas esperadas no CSV:
    t, theta_ref_deg, theta_deg, erro_deg,
    torque_cmd_Nm, torque_est_Nm, corrente_A, temp_C
"""
import sys
import argparse
import csv
import os


def ler_csv(caminho):
    dados = {k: [] for k in
             ["t", "theta_ref_deg", "theta_deg", "erro_deg",
              "torque_cmd_Nm", "torque_est_Nm", "corrente_A", "temp_C"]}
    with open(caminho) as f:
        leitor = csv.DictReader(f)
        for linha in leitor:
            for k in dados:
                try:
                    dados[k].append(float(linha[k]))
                except (ValueError, KeyError):
                    dados[k].append(float("nan"))
    return dados


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="arquivo CSV do logger_node")
    ap.add_argument("--salvar", default=None,
                    help="pasta para salvar as figuras (PNG)")
    args = ap.parse_args()

    try:
        import matplotlib
        if args.salvar:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib nao instalado. Instale com: pip install matplotlib")
        sys.exit(1)

    d = ler_csv(args.csv)
    if not d["t"]:
        print("CSV vazio ou invalido.")
        sys.exit(1)

    t = d["t"]
    print(f"Lidas {len(t)} amostras, duracao {t[-1]-t[0]:.2f} s")

    # ---- Figura 1: rastreio (referencia vs medido) + erro ----
    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax1.plot(t, d["theta_ref_deg"], "k--", lw=1.4, label="referencia")
    ax1.plot(t, d["theta_deg"], "b", lw=1.0, label="medido")
    ax1.set_ylabel("angulo [graus]")
    ax1.set_title("Rastreio de posicao")
    ax1.legend(); ax1.grid(alpha=0.3)

    ax2.plot(t, d["erro_deg"], "r", lw=1.0)
    ax2.set_ylabel("erro [graus]")
    ax2.set_xlabel("tempo [s]")
    ax2.set_title("Erro de rastreio")
    ax2.grid(alpha=0.3)
    fig1.tight_layout()

    # ---- Figura 2: torque e corrente ----
    fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax3.plot(t, d["torque_cmd_Nm"], "g", lw=1.0, label="torque comandado")
    ax3.plot(t, d["torque_est_Nm"], "m", lw=0.8, alpha=0.8,
             label="torque estimado (corrente)")
    ax3.set_ylabel("torque [N.m]")
    ax3.set_title("Torque")
    ax3.legend(); ax3.grid(alpha=0.3)

    ax4.plot(t, d["corrente_A"], "c", lw=1.0)
    ax4.set_ylabel("corrente [A]")
    ax4.set_xlabel("tempo [s]")
    ax4.set_title("Corrente")
    ax4.grid(alpha=0.3)
    fig2.tight_layout()

    # ---- Figura 3: deteccao de tremor (espectro do torque) ----
    # Se houver tremor, aparece um pico de frequencia no espectro.
    fig3, ax5 = plt.subplots(1, 1, figsize=(11, 4))
    try:
        import numpy as np
        tq = np.array(d["torque_cmd_Nm"])
        tq = tq - np.nanmean(tq)
        n = len(tq)
        if n > 10 and t[-1] > t[0]:
            fs = n / (t[-1] - t[0])  # taxa de amostragem media
            freqs = np.fft.rfftfreq(n, d=1.0/fs)
            espectro = np.abs(np.fft.rfft(tq))
            ax5.plot(freqs, espectro, "b", lw=0.9)
            ax5.set_xlabel("frequencia [Hz]")
            ax5.set_ylabel("amplitude")
            ax5.set_title("Espectro do torque (picos = tremor/oscilacao)")
            ax5.set_xlim(0, min(fs/2, 100))
            ax5.grid(alpha=0.3)
            # marca o pico dominante
            if len(espectro) > 2:
                idx = int(np.argmax(espectro[1:]) + 1)
                ax5.axvline(freqs[idx], color="r", ls=":",
                            label=f"pico ~{freqs[idx]:.1f} Hz")
                ax5.legend()
                print(f"Pico de frequencia no torque: {freqs[idx]:.1f} Hz")
                print("(se for alto, ~dezenas de Hz, confirma o tremor)")
    except ImportError:
        ax5.text(0.5, 0.5, "numpy necessario para o espectro",
                 ha="center", transform=ax5.transAxes)
    fig3.tight_layout()

    # ---- saida ----
    if args.salvar:
        os.makedirs(args.salvar, exist_ok=True)
        fig1.savefig(os.path.join(args.salvar, "rastreio.png"), dpi=130)
        fig2.savefig(os.path.join(args.salvar, "torque_corrente.png"), dpi=130)
        fig3.savefig(os.path.join(args.salvar, "espectro_tremor.png"), dpi=130)
        print(f"Figuras salvas em {args.salvar}/")
    else:
        plt.show()


if __name__ == "__main__":
    main()
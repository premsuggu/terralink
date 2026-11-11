"""Turn the printed table of eval_exploration.py (one or more worlds, in the order given by --worlds) into a markdown table and a bar chart.

usage: python3 summarize_eval.py RAW.log OUT_PREFIX --worlds w4 w5 w2 w1 w3 --a adopted_underroof --b adopted
Reads rows `map policy goal reached driven shortest ratio plans switches ...`; policy `--a` is the baseline, `--b` the new one.
"""
import argparse
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse(path, worlds):
    rows, w = [], -1
    for line in open(path):
        if line.startswith("map "):
            w += 1
        m = re.match(r"^(\S+)\s+(\S+)\s+(\S+)\s+(True|False)\s+([\d.]+)\s+(\S+)\s+\S+\s+(\d+)\s+(\d+)", line)
        if m and w >= 0:
            rows.append(dict(world=worlds[w], policy=m[2], goal=m[3], ok=m[4] == "True", driven=float(m[5]), plans=int(m[7]), switches=int(m[8])))
    return rows


def main():
    a = argparse.ArgumentParser()
    a.add_argument("raw"); a.add_argument("out")
    a.add_argument("--worlds", nargs="+", required=True)
    a.add_argument("--a", required=True); a.add_argument("--b", required=True)
    args = a.parse_args()
    rows = parse(args.raw, args.worlds)
    byw = {}
    for r in rows:
        byw.setdefault(r["world"], {}).setdefault(r["policy"], {})[r["goal"]] = r
    md = [f"| world | goals | {args.a}: reached | {args.a}: switches | {args.b}: reached | {args.b}: switches |", "|---|---|---|---|---|---|"]
    tot = {args.a: [0, 0, 0], args.b: [0, 0, 0]}
    for w in args.worlds:
        cells = []
        n = len(byw[w][args.a])
        for p in (args.a, args.b):
            rs = byw[w][p].values(); ok = sum(r["ok"] for r in rs); sw = sum(r["switches"] for r in rs)
            cells += [f"{ok}/{n}", str(sw)]
            tot[p][0] += ok; tot[p][1] += n; tot[p][2] += sw
        md.append(f"| {w} | {n} | " + " | ".join(cells) + " |")
    md.append(f"| **all** | {tot[args.a][1]} | {tot[args.a][0]}/{tot[args.a][1]} | {tot[args.a][2]} | {tot[args.b][0]}/{tot[args.b][1]} | {tot[args.b][2]} |")
    open(args.out + ".md", "w").write("\n".join(md) + "\n")
    print("\n".join(md))
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    x = range(len(args.worlds))
    for i, p in enumerate((args.a, args.b)):
        ax[0].bar([k + 0.4 * i for k in x], [sum(r["ok"] for r in byw[w][p].values()) for w in args.worlds], 0.4, label=p)
        ax[1].bar([k + 0.4 * i for k in x], [sum(r["switches"] for r in byw[w][p].values()) for w in args.worlds], 0.4, label=p)
    for axx, t in zip(ax, ("goals reached", "target switches (lower = steadier)")):
        axx.set_xticks([k + 0.2 for k in x]); axx.set_xticklabels([w.split("_")[0] for w in args.worlds]); axx.set_title(t)
    ax[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(args.out + ".png", dpi=130)


if __name__ == "__main__":
    main()

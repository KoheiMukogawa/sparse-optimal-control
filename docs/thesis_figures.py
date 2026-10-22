# -*- coding: utf-8 -*-
"""卒論の図番号 → 画像パスの対応表（HTML版・LaTeX版で共用）。

`results/` は .gitignore 対象なので、ここに載っていてもローカルに
画像が無いことがある。各ビルダは存在しないパスを「未取得」扱いにする。
"""

# 図番号 → 画像パス（リポジトリルートからの相対）。
# 【未取得】の図（実機写真・積載）はここに無く、本文には枠だけ出る。
FIGURES = {
    "1.2": "docs/論文解説_figs/fig_compare.png",
    "1.3": "results/2026-08-02_thesis_figs2/fig1_3_logic_flow.png",
    "2.1": "docs/論文解説_figs/fig_geom.png",
    "2.2": "docs/論文解説_figs/fig_norms.png",
    "2.3": "docs/論文解説_figs/fig_bob.png",
    "2.4": "docs/論文解説_figs/fig_soft.png",
    "2.5": "results/2026-07-26_openloop_bangoffbang/bangoffbang.png",
    "3.1": "results/2026-08-02_thesis_figs2/fig3_1_cost_diff.png",
    "3.2": "results/2026-08-02_thesis_figs2/fig3_2_move_suppress.png",
    "3.3": "results/2026-08-02_thesis_figs2/fig3_3_node.png",
    "4.1": "results/2026-08-02_thesis_figs2/fig4_1_system.png",
    "4.3": "results/2026-08-02_thesis_figs2/fig4_3_layers.png",
    "4.4": "results/2026-07-16_Lturn_1m_smoke2/traj_odom_vs_truth.png",
    "4.5": "results/2026-08-02_thesis_figs2/fig4_5_geometry.png",
    "4.6": "results/2026-08-02_thesis_figs2/fig4_6_detection.png",
    "4.7": "results/2026-08-02_thesis_figs2/fig4_7_tag_layout.png",
    "4.8": "results/2026-08-02_thesis_figs2/fig4_8_survey.png",
    "4.9": "results/2026-08-02_thesis_figs2/fig4_9_homing.png",
    "4.10": "results/2026-08-02_thesis_figs2/fig4_10_batch.png",
    "5.1": "results/2026-08-02_thesis_figs2/fig5_1_solve_vs_N.png",
    "5.2": "results/2026-06-13_lambda_sweep/tradeoff.png",
    "5.3": "results/2026-06-13_Lturn_plot.png",
    "5.4": "results/2026-07-26_delay_sweep/delay_sweep.png",
    "5.5": "results/2026-07-26_lambda_ms_grid/sweep_ms.png",
    "5.6": "results/2026-07-26_thesis_figs/fig5_6_omega_before_after.png",
    "5.7": "results/2026-07-26_thesis_figs/fig5_7_real_vs_sim.png",
    "5.8": "results/2026-08-02_thesis_figs2/fig5_8_summary.png",
    "6.1": "results/2026-08-02_thesis_figs2/fig6_1_course.png",
    "6.2": "results/2026-07-26_thesis_figs/fig6_2_rmse_sumu.png",
    "6.3": "results/2026-07-26_thesis_figs/fig6_3_omega_all.png",
    "6.4": "results/2026-07-26_thesis_figs/fig6_4_solve_time.png",
    "6.5": "results/2026-07-26_thesis_figs/fig6_5_endpoint_scatter.png",
    "6.6": "results/2026-07-26_thesis_figs/fig6_6_odom_vs_truth.png",
    "6.8": "results/2026-07-26_thesis_figs/fig6_8_radar.png",
    "7.1": "results/2026-08-02_thesis_figs2/fig7_1_limit_cycle.png",
    "7.2": "results/2026-08-02_thesis_figs2/fig7_2_apparent_optimum.png",
}

# 実機が要るため未取得の図。本文には「未取得」の枠を出す。
MISSING = {
    "1.1": "LiteRover の外観と搭載機器（実機復旧後に撮影）",
    "4.2": "実験環境の全景（実機復旧後に撮影）",
    "6.7": "積載質量に対する指標の推移（外乱バッチ R1 待ち）",
}

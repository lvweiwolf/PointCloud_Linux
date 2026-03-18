系统安装包：
sudo apt install libomp-dev (Clang)
sudo apt install libpugixml-dev
sudo apt install libgmp-dev libmpfr-dev
sudo apt install libpcap-dev
sudo apt install libgoogle-glog-dev


层级0 的 PieceInfo #5  ─┐
                        ├─→ 冲突！被错误地归到同一个 bucket
层级2 的 PieceInfo #5  ─┘

nLevelIndex (4 bits)              nsize (48 bits)
┌──────┐        ┌────────────────────────────────────────┐
│ 0010 │  <<48  │ 0000 0000 0000 0000 0000 0000 0000 0101 │
└──────┘        └────────────────────────────────────────┘
        │                           │
        └─────────── OR ────────────┘
                    │
                    ▼
┌──────┬────────────────────────────────────────┐
│ 0010 │ 0000 0000 0000 0000 0000 0000 0000 0101 │
└──────┴────────────────────────────────────────┘
    高16位                    低48位

为什么是 48 位？
size_t 通常是 64 位，层级最多 4 个（只需 2 位），48 位足够容纳 nsize（最大约 2.8 × 10¹⁴）。
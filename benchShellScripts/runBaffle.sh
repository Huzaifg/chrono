./demo_FSI_BaffleFlowTest --ps_freq 1 --output true
./demo_FSI_BaffleFlowTest --ps_freq 2 --output true
./demo_FSI_BaffleFlowTest --ps_freq 4 --output true
./demo_FSI_BaffleFlowTest --ps_freq 8 --output true

./demo_FSI_BaffleFlowTest  --ps_freq 1 --output true
./demo_FSI_BaffleFlowTest  --ps_freq 2 --output true
./demo_FSI_BaffleFlowTest  --ps_freq 4 --output true
./demo_FSI_BaffleFlowTest  --ps_freq 8 --output true

## Now for benchmarking speed
mkdir -p BaffleFlowPerformance
./demo_FSI_BaffleFlowTest  --ps_freq 1 --output false --no_vis true --snapshots false > BaffleFlowPerformance/1_1.log
./demo_FSI_BaffleFlowTest  --ps_freq 2 --output false --no_vis true --snapshots false > BaffleFlowPerformance/2_1.log
./demo_FSI_BaffleFlowTest  --ps_freq 4 --output false --no_vis true --snapshots false > BaffleFlowPerformance/4_1.log
./demo_FSI_BaffleFlowTest  --ps_freq 8 --output false --no_vis true --snapshots false > BaffleFlowPerformance/8_1.log

./demo_FSI_BaffleFlowTest  --ps_freq 1 --output false --no_vis true --snapshots false --size_factor 2 > BaffleFlowPerformance/1_2.log
./demo_FSI_BaffleFlowTest  --ps_freq 2 --output false --no_vis true --snapshots false --size_factor 2 > BaffleFlowPerformance/2_2.log
./demo_FSI_BaffleFlowTest  --ps_freq 4 --output false --no_vis true --snapshots false --size_factor 2 > BaffleFlowPerformance/4_2.log
./demo_FSI_BaffleFlowTest  --ps_freq 8 --output false --no_vis true --snapshots false --size_factor 2 > BaffleFlowPerformance/8_2.log

./demo_FSI_BaffleFlowTest  --ps_freq 1 --output false --no_vis true --snapshots false --size_factor 3 > BaffleFlowPerformance/1_3.log
./demo_FSI_BaffleFlowTest  --ps_freq 2 --output false --no_vis true --snapshots false --size_factor 3 > BaffleFlowPerformance/2_3.log
./demo_FSI_BaffleFlowTest  --ps_freq 4 --output false --no_vis true --snapshots false --size_factor 3 > BaffleFlowPerformance/4_3.log
./demo_FSI_BaffleFlowTest  --ps_freq 8 --output false --no_vis true --snapshots false --size_factor 3 > BaffleFlowPerformance/8_3.log



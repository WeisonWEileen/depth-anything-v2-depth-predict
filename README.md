# depth_calculate
## 一些比较稳定的 深度预测的点
index : 
79 33 30 56

## run the demo:
rgb tif and txt files path are configured in ```${workspacefolder}/config/filePath.yaml```. Modified it then run the programme. In the project root dir.
```
cmake -B build
cmkae --build build
./build/depth_calculator
```
## preprerequisite
- opencv
- eigen
- png++ (download the tar.gz package and run sudo make install)

### TODO
为什么算出来是负数就更加逆天了，我觉得坐标变换应该没有错的啊
$$
^iT P_{world}= {^{i}P_{cam}}\\
{^{i+1}T_i} P_{world}={^{i+1}P_{cam}}\\
{^iT\cdot(^{i+1}T)^{-1}\cdot{^{i+1}P_{cam}}}={^{i}P_{cam}}
$$
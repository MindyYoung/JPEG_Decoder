A JPEG Decoder
===================

# 1.功能

输入JPEG压缩图（.jpeg），至解码器后输出原始图像（.bmp）


# 2.编码过程

 1. RGB格式转换为YUV格式
 2. 将图像8×8分块
 3. 正向离散余弦变换(FDCT)
 4. Z字形存储与读取(zigzag scan)
 5. 量化(quantization)
 6. 使用差分脉冲编码调制(DPCM)对直流系数(DC)进行编码
 7. 使用行程长度编码(RLE)对交流系数(AC)进行编码
 8. 熵编码

# 3.解码过程

 1. 读入文件的相关信息
 2. 对Huffman表读出和建立Huffman树
 3. Huffman解码
 4. 反zigzag扫描
 5. 反量化(量化表反zigzag扫描)
 6. 反DCT变换
 7. YCrCb模式向RGB模式转换


# 4.使用说明
- C++，vs2012
- 打开JPEG_Decoder.sln，生成exe
- 在JPEG_Decoder\JPEG_Decoder替换input.jpg（压缩图像）

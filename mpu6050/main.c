#include <stdio.h>  
#include <math.h>  
  
#define TABLE_SIZE 500 // 查找表的大小  
//#define RANGE_RAD (M_PI*2) // 我们关注的弧度范围，例如0到π/2  
#define QUANTIZATION (2.0f/TABLE_SIZE) // 量化步长  
  
double atan_table[TABLE_SIZE];  
  
void generate_atan_table() {  
    for (int i= 0; i <= TABLE_SIZE; i++) {      
	double rad = i * QUANTIZATION -1.0f; // 计算当前索引对应的弧度  
        //double deg = rad * (180.0 / M_PI); // 将弧度转换为度  
        atan_table[i] = asin(rad); // 存储转换后的hudu值

	atan_table[i] *= (180.0f/M_PI);  
	//printf("%f, %f, ", rad, atan_table[i]);
    }  
}  
  
int main() {  
    generate_atan_table();  
    printf("rad step=%f\r\n", QUANTIZATION);    

    printf("// 反正切函数查找表（弧度索引，以度为单位），从atan(0)到atan(π/2)的近似值\n");  
    printf("// 注意：这只是一个示例，实际使用中可能需要根据需要调整量化步长和查找表大小\n");  
	
    for (int i = 0; i <= TABLE_SIZE; i++) {  
        if(i%20==0)
	{
	    printf("\r\n");
	}

        if (i < TABLE_SIZE - 1) {  
            printf("%.2f, ", atan_table[i]);  
        } else {  
            printf("%.2f\n", atan_table[i]);  
        }  
    }  

    fflush(stdout);
    return 0;  
}

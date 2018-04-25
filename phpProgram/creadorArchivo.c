#include<stdio.h>
#include<stdlib.h>
#include<time.h>
int main(int argc,char *argv[]){
	char nombreArchivo[100] = "blimp01.eko";
	srand(time(NULL)); 
	float contador = 0;
	for (nombreArchivo[6]='1';nombreArchivo[6]<='3';nombreArchivo[6]++){
		FILE * archivo = fopen(nombreArchivo,"w");
		if (archivo == NULL) {
			printf("Problemas para abrir archivo\n");
			return 1;
		}
		int tipo = 0;

		for (int tipo = 0; tipo < 5; ++tipo)
		{
			contador = 0;
			float valor = (rand()%10)+contador;
			for (int i = 0; i < 2000; ++i)
			{

			fwrite(&tipo,sizeof(int),1,archivo);
			fwrite(&valor,sizeof(float),1,archivo);
			valor = (rand()%10)+contador;
			contador = contador + 3;
			}
			if (tipo == 0)
			{
				printf("%f\n",valor);
			}

		}
		
		fclose(archivo);

	}
	return 0;
}
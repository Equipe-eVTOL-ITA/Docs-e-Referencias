import yaml
from math import cos, radians
import numpy as np
import matplotlib.pyplot as plt


class ProjetoBateria:

    def __init__(self):
        with open('eletrica/src/config/componentes.yaml', 'r') as f:
            self.data = yaml.safe_load(f)

        self.eletricos = []
        self.tempo = 1
        self.outros = []
        self.payload = 0.0
        self.payload_tempo = 0.0
        self.theta = radians(10)
        self.carga_min = 2000
        self.carga_max = 5000
        self.inicializar()

    def inicializar(self):
        self.tempo = float(input("Defina o tempo de voo: "))
        self.carga_min = float(input("Carga mínima para plot: "))
        self.carga_max = float(input("Carga máxima para plot: "))
            
    def definir_payload(self):
        self.payload = float(input("Peso da carga paga: "))
        if self.payload == 0.0:
            return
        self.payload_tempo = float(input("Tempo de voo com carga paga: "))


    def escolher_eletricos(self):

        for nome in self.data['eletricos'].keys():
            res = input(nome + ' (s/n): ')
            if res in 'sS':
                new_dict = self.data['eletricos'][nome].copy()
                new_dict['nome'] = nome
                self.eletricos.append(new_dict)

    def escolher_outros(self):

        for categoria, categoria_dict in self.data['outros'].items():
        
            if len(categoria_dict.keys()) == 1:
                first_key = next(iter(categoria_dict))
                new_dict = {'nome': categoria + ' ' + first_key,
                            'peso': categoria_dict[first_key]['peso']}
                self.outros.append(new_dict)
                continue

            print(f'Escolha de {categoria}?')

            for modelo, modelo_dict in categoria_dict.items():
                print(f"  > Modelo: {modelo}")
                print(f"    - Peso: {modelo_dict['peso']}g")
            escolha = input('modelo escolhido: ')

            if escolha in categoria_dict.keys():
                new_dict = {'nome': categoria + ' ' + escolha, 
                            'peso': categoria_dict[escolha]['peso']}
                self.outros.append(new_dict)

    def calcular(self):

        self.modelos = dict()
        motores = self.data['motores']
        for motor in motores:
            self.modelos[motor['modelo']] = dict()

        self.calcular_coeficiente_motores()
        self.calcular_massa()
        self.calcular_correntes()

    def resultados(self):

        self.calcular()

        plt.figure(figsize=(10,6))
        plt.axhline(y=self.tempo, color='lightgray', linestyle='--', label=f'Tempo = {self.tempo} min')


        for nome_modelo, dic in self.modelos.items():

            x_arr = dic['simulation']['carga']
            # Retira a carga gasta durante o payload
            # e calcula o tempo de voo (em horas) sem o payload
            y_arr = (x_arr - dic['simulation']['corrente_media_payload'] * 1000 * self.payload_tempo/60) \
                /(dic['simulation']['corrente_media'] * 1000)
            y_arr *= 60 # converte para minutos
            y_arr += self.payload_tempo # adiciona o tempo de voo com payload

            # Encontra o valor da carga relacionado à autonomia mais próxima do valor desejado
            diffs = np.abs(y_arr - self.tempo)
            min_idx = np.argmin(diffs)
            carga_alvo = x_arr[min_idx]
            corrente_max = dic['simulation']['corrente_max']
            c = corrente_max / carga_alvo * 1000
            
            plt.plot(x_arr, y_arr, label=f"{nome_modelo} - {dic['voltagem']}V")
            plt.scatter(carga_alvo, y_arr[min_idx])
            
            print(f"{nome_modelo} - {dic['voltagem']}V")
            print(f"Carga: {carga_alvo:.0f}mAh")
            print(f"Corrente máxima: {corrente_max:.1f}A")
            print(f"C: {c}\n")

        plt.xlabel('Carga da bateria')
        plt.ylabel('Tempo de voo')
        plt.title('Autonomia do drone em função da carga da bateria')
        plt.legend()
        plt.show()


    def calcular_coeficiente_motores(self):
        motores = self.data['motores']
        for motor in motores:
            k = (motor['thrust_max'])**3 / (motor['potencia_max'])**2
            motor['k'] = k

    def calcular_massa(self):

        # Peso base para todas iterações de motores
        # considera os componentes elétricos e os componentes de peso não desprezível
        base = sum([item['peso'] for item in self.outros])
        base += sum([item['peso'] for item in self.eletricos])

        motores = self.data['motores']
        for motor in motores:
            # Adiciona o peso dos 4 motores ao drone
            peso = base + 4 * motor['peso']
            # Registra alguns valores nos modelos testados
            voltagem = motor['voltagem']
            self.modelos[motor['modelo']]['peso'] = peso
            self.modelos[motor['modelo']]['voltagem'] = voltagem

    def calcular_correntes(self):
        # Cálculo da potencia/corrente necessária para alimentar os componenentes elétricos
        potencia_media_base = 0
        potencia_max_base = 0
        for componente in self.eletricos:
            # Media
            pot = componente['corrente_media'] * componente['voltagem']
            potencia_media_base += pot
            # Maxima
            pot = componente['corrente_max'] * componente['voltagem']
            potencia_max_base += pot

        # Cálculo de potencia/corrente para os diferentes motores
        for motor in self.data['motores']:

            nome_modelo = motor['modelo']
            k = motor['k']
            voltagem = self.modelos[nome_modelo]['voltagem']

            carga_especifica = self.data['energia_especifica'] / voltagem # transformando Wh/kg em mAh/g
            bateria_carga = np.arange(self.carga_min, self.carga_max+1) # cria um array de inteiros para simular diferentes cargas
            bateria_massa = bateria_carga / carga_especifica

            massa = self.modelos[nome_modelo]['peso'] + bateria_massa # adiciona a massa da bateria ao modelo

            ###### 
            self.modelos[nome_modelo]['simulation'] = dict()
            self.modelos[nome_modelo]['simulation']['carga'] = bateria_carga
            self.modelos[nome_modelo]['simulation']['bateria_massa'] = bateria_massa

            ###### Sem payload
            # media
            pot = 4 * (massa / (4 * np.cos(self.theta)))**1.5 / k**0.5
            self.modelos[nome_modelo]['simulation']['potencia_media'] = potencia_media_base + pot
            self.modelos[nome_modelo]['simulation']['corrente_media'] = \
                self.modelos[nome_modelo]['simulation']['potencia_media'] / voltagem

            # maxima
            pot = 4 * motor['potencia_max']
            self.modelos[nome_modelo]['simulation']['potencia_max'] = potencia_max_base + pot
            self.modelos[nome_modelo]['simulation']['corrente_max'] = 4*motor['corrente_max'] + potencia_max_base/voltagem

            ###### com payload
            # media
            pot = 4 * ((massa+self.payload)  / (4 * cos(self.theta)))**1.5 / k**0.5
            self.modelos[nome_modelo]['simulation']['potencia_media_payload'] = potencia_media_base + pot
            self.modelos[nome_modelo]['simulation']['corrente_media_payload'] = self.modelos[nome_modelo]['simulation']['potencia_media_payload'] / voltagem

                        
if __name__ == '__main__':

    print("\n##################\
           \n## TEMPO DE VOO ##\
           \n##################\n")
    bateria = ProjetoBateria()

    print("\n#################\
            \n## CARGA PAGA ##\
            \n#################\n")
    bateria.definir_payload()


    print("\n###########################\
           \n## COMPONENTES ELÉTRICOS ##\
           \n###########################\n")
    bateria.escolher_eletricos()

    print("\n########################\
           \n## OUTROS COMPONENTES ##\
           \n########################\n")
    bateria.escolher_outros()

    print("\n#################\
            \n## RESULTADOS ##\
            \n#################\n")
    bateria.resultados()
    

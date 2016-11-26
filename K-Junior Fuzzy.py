import vrep

vrep.simxFinish(-1) # Fecha conexões anteriores

#Conectando com o V-rep
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Faz conexão, se retorna -1 é que deu erro

if clientID!=-1:
    print('Conexão sucedida')
else:
    print("Erro na conexão")

#Declarando variáveis

sensores_handle = []        #lista com os Handles dos sensores, para pegar os Handles mais fácil
valores_sensores = []       # Lista com os valores dos sensores
v_left = 0                  # velocidade motor esquerdo, vai de 0 à 10 (eu determinei impiricamente na simulação)
v_right = 0                 # idem ao de cima pro motor direito


#Declaração de Handles

#handle do motor esquerdo, chuto que o nome do objeto seja o mesmo da simulação
erro_code, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'KJunior_wheelLeft', vrep.simx_opmode_oneshot_wait )

#Handle do motor Direito
erro_code, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'KJunior_wheelLeft', vrep.simx_opmode_oneshot_wait )

#Handle dos sensores, só importam os do 1 ao 5, o 3 é o sensor mais centralizado
for i in range(5):
    erro_code, sensor_handle = vrep.simxGetObjectHandle(clientID,'KJunior_proxSensor%d' % (i+1), vrep.simx_opmode_oneshot_wait)
    sensores_handle.append(sensores_handle)
    # Não sei porque, mas a primeira leitura tem que ser assim
    error_code,estado,coord,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, sensores_handle[i],vrep.simx_opmode_streaming)


# Lógica em si:

#Lê os sensores do robo e os coloca na lista de valores
#erro_code,


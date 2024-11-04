# kit_encoder_driver

Esse é o pacote ROS do Kit de Robótica dedicado à transformar as informações de encoders em mensagens ROS e calcular Odometria e Transformadas a partir disso.

## Dependencias

Clone esse pacote dentro do seu ROS workspace, dentro da pasta src. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/
git clone git@github.com:pedro-fuoco/kit_encoder_driver.git
```

### Rosdep
Esse pacote foi desenvolvido para a versão ROS 2 Jazzy. Uma vez que o ROS estiver devidamente instalado e inicializado, as dependencias especificas desse pacote podem ser instaladas através do rosdep.

Rosdep é um meta-package manager, utilizado para facilitar a instalação de dependencias. Apesar do nome com base histórica, essa ferramenta se tornou independente do ROS e deve ser instalada separadamente:

```bash
sudo apt install python3-rosdep
```

Uma vez instalado, ele deve ser inicializado e atualizado:

```bash
sudo rosdep init
rosdep update
```

Por fim, para esse repositorio, vamos utilizar o rosdep para instalar as dependencias listadas no arquivo `package.xml`. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/kit_encoder_driver
rosdep install --from-paths . -y --ignore-src
```

### Instalação manual de bibliotecas
Infelizmente, uma das bibliotecas utilizadas nesse pacote não está nos repositorios de indice do rosdep. Por conta disso, precisamos instala-la manualmente seguindo o seguinte comando:

```bash
sudo apt install python3-rpi-lgpio
```

### Colcon
Inicialize o seu ROS workspace e compile ele utilizando a ferramenta `colcon`. Mais uma vez, é necessario substituir o endereço e nome do seu próprio ROS workspace abaixo:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_encoder_driver
```
Fique de olho em possiveis erros nesse processo e realize debug se necessario.

### Hardware
Esse pacote assume que são utilizados dois encoders de quadratura, em um robô de duas rodas. Porém cada um dos nodes foi feito de forma modular, e um tipo diferente de encoder pode ser facilmente substituido com a reimplementação do arquivo `encoder_node.py`, ou até mesmo outra plataforma robótica pode ser utilizada, ao trocar os calulos feitos em `odometry_node`.

Vale ressaltar que os pinos de entrada de cada um dos encoders e as dimensões da plataforma robótica são configuraveis.

## Configurações
As configurações desse pacote são feitas através de ROS `params`. Os arquivos que descrevem as configurações, que podem ser editados manualmente, se encontram na pasta `config`.
Lembre-se sempre de recompilar o pacote depois de fazer mudanças na configuração, com:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_encoder_driver
```

## Launch
Para iniciar os programas `encoder_node`, `odometry_node`, responsaveis por publicar os ticks dos encoders e calcular a odometria das rodas, respectivamente, basta utilizar o seguinte comando:
```bash
ros2 launch kit_encoder_driver odometry_launch.py
```

Para facilitar o teste desse pacote, foi criado um modo de `debug`, no qual a transformação entre os eixos `base_link` e `odom` é publicada diretamente pelo node `transformations_node`. Isso permite que a odometria publicada seja visualizada em ferramentas como o `rviz2`. Dito isso, esse modo deve ser desabilitado sempre que outro pacote for responsável por publicar a transformação dos eixos do robô. Para habilita-lo, base rodar o seguinte comando:
```bash
ros2 launch kit_encoder_driver odometry_launch.py debug:=true
```
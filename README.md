# Aula de gazebo - 2023

## Descrição :  
 
Esse pacote tem o intuito de auxiliar as aulas de Gazebo/Ros do ano de 2023

## Como instalar : 

Para instalar o pacote execute o comando do git clone no seu Workspace :

> cd sua_work_space/src
> 
> git clone https://github.com/gelardrc/aula_gazebo.git
>
> cd ..
> 
> catkin_make

resultado esperado : 

![image](https://drive.google.com/uc?export=view&id=1HBmUMmLUaIsdPd3a1qtdkE1yonNBCkRr)

Agora copie e cole o arquivo aula_setup.bash para a pasta sua_workspace/devel (isso pode ser feito por meio do navegador de arquivos do ubuntu ou pelo comando mv no terminal).

**(o comando deve ser realizado dentro da pasta aula_gazebo)**

> mv aula_setup.bash ../../devel/

Todas às vezes que abrir uma nova aba no terminal você tera que executar o seguinte comando: 

> source sua_work_space/devel/aula_setup.bash

**Esse comando garante que o ROS e o Gazebo estarão com as variáveis de ambiente corretas.**

_________________________________________________________________________________________________


## Testando o pacote : 

Após realizar o source devel/aula_setup.bash utilize o roslaunch para iniciar uma simulação de mundo.

> roslaunch aula_gazebo  

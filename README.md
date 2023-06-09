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

Agora copie e cole os arquivos da pasta model para a pasta .gazebo/models (isso pode ser feito por meio do navegador de arquivos do ubuntu ou pelo comando mv no terminal).

**(o comando deve ser realizado dentro da pasta aula_gazebo)**

> mv models/* ~/.gazebo/models

**obs:** Para vizualizar pastas que comecem com .<nome_da_pasta> (ocultas) no linux, vá ate o local da pasta e clique em crtl+h. Caso a pasta models nao exista dentro de .gazebo, basta você cria-la.


Todas às vezes que abrir uma nova aba no terminal você tera que executar o seguinte comando: 

> source sua_work_space/devel/setup.bash

**Esse comando garante que o ROS e o Gazebo estarão com as variáveis de ambiente corretas.**

_________________________________________________________________________________________________


## Testando o pacote : 

Após realizar o source devel/setup.bash utilize o roslaunch para iniciar uma simulação de mundo.

> roslaunch aula_gazebo robot_gazebo.launch 

# aula_gazebo
Aula de gazebo - 2023

Descrição :  Esse pacote tem o ituito de auxiliar as aulas de Gazebo/Ros do ano de 2023

________________________________________________________________________________________________

Como instalar : Para instalar o pacote execute o comando do git clone no seu Workspace 

> git clone 

Agora copie e cole o arquivo aula_setup.bash para a a pasta sua_workspace/devel ( isso pode ser feito por meio do navegador de arquivos do ubunto ou pelo comando mv no terminal )

( comando deve ser realizado dentro da pasta aula_gazebo )

> mv aula_setup.bash ../../devel/

Todas as vezes que abrir uma nova aba no terminal voce tera que executar o seguinte comando : 

> source sua_work_space/devel/aula_setup.bash

# Esse comando garante que o ROS e o Gazebo estarão com as variáveis de ambiente corretas.

_________________________________________________________________________________________________


Testando o pacote : 

Após realizar o source devel/aula_setup.bash utilize o roslaunch para iniciar uma simulação de mundo


> roslaunch aula_gazebo  

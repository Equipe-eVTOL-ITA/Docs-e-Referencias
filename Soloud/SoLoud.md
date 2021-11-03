# Biblioteca Soloud

A SoLoud é uma biblioteca para reprodução programática de sons em C++. A biblioteca é distribuida na forma de arquivos fonte, então não tem nenhum `.dll` ou `.so`. Os códigos-fonte podem ser adicionados ao repositório do projeto. Páginas úteis:
> [Site oficial](https://sol.gfxile.net/soloud/index.html)  
> [Download](https://sol.gfxile.net/soloud/downloads.html)  
> [Documentação completa](https://sol.gfxile.net/soloud/soloud_20200207.html)

## Como incluir a SoLoud em um projeto

As pastas relevantes para o uso da SoLoud em código são a `include` e a `src`. A pasta `include` contém os headers correspondentes aos `.cpp` da pasta `src`. Existem três pastas relevantes aqui:

* `audiosource`: Contém os códigos-fonte das várias formas de input de áudio. Por exemplo, para ler arquivos `.wav`, é preciso linkar os arquivos `soloud_wav.cpp`, `dr_impl.cpp` e `stb_vorbis.c` da pasta `wav`. Naturalmente, o código deve incluir o header `soloud_wav.h` da pasta dos includes. **OBS**: incluir mais de uma fonte de áudio pode dar problema com o `stb_vorbis.c`, que não tem header guard (!!!)

* `backend`: Contém a interface com os drivers de áudio, sistema operacional, etc. É necessário escolher ao menos um backend, compatível com a máquina que vai rodar o código, para a SoLoud funcionar. Na compilação, usar a flag `-Dnome_do_backend` ativa esse backend. Com o CMake, é possível usar os comandos `add_definitions`ou `target_compile_options` para fazer isso. Apesar de ser uma definição de pré-processador, um simples `#define` no código não funciona. Vai entender. Para mais informações, ver os arquivos do CMake da pasta `contrib`.  
  
* `core`: Contém toda a funcionalidade da SoLoud. A princípio, nem todos os arquivos daqui são necessários para um projeto funcionar. Mas sugou ficar escolhendo, então linkar todos eles é o padrão. No `contrib/src.CMAKE` tem uma lista bonitinha de todos os arquivos dessa pasta. Também seria possível usar o comando `file` do CMake para incluir todos os arquivos de uma pasta, mas é má prática.

## Observações gerais

* A SoLoud executa áudio em uma thread separada. Isso quer dizer que o nosso código continua a execução durante a reprodução do áudio.  
* Não esquecer de chamar as funções `init` e `deinit`.
* Esse arquivo contém informações mocadas que a documentação não explica direito. Para mais detalhes sobre o funcionamento da SoLoud, veja a documentação completa. Recomendo começar com as classes `SoLoud::SoLoud` e `SoLoud::Wav`.

## TODO

Ainda não sabemos tudo sobre a SoLoud, nem as melhores formas de lidar com ela. Problemas em aberto:  

* Adicionar o código da SoLoud no nosso repo é bagunçado e deixa o repositório grande. Ver como usar um `submodule` do git
* Fazer um .o da SoLoud, com o audiosource e o backend corretos pro projeto

Autor: Pulga (T24)
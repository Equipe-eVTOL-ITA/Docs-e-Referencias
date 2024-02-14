# DJI Mobile SDK Setup

A DJI providencia um Mobile Software Development Kit (SDK) como uma interface de desenvolvimento de aplicativos Android ou iOS que controlem drones DJI.

Podemos usar o Sample Code do Mobile SDK como base para um aplicativo com funcionalidades básicas e então desenvolver o back-end do aplicativo para tarefas de visão computacional e controle de voo que utilizem hardware da DJI.

Siga os passos em: https://github.com/dji-sdk/Mobile-SDK-Android.git

O tutorial é antigo, então você encontrará alguns problemas de versionamento de Gradle e Java ao abrir o projeto no Android Studio.

Para resolver os problemas:

1. `Android Studio error "Installed Build Tools revision 31.0.0 is corrupted"`

    Isso ocorre pois o Java procura um arquivo dx, mas ele está como d8. Resolução:
```shell
    # change below to your Android SDK path
    cd ~/Library/Android/sdk/build-tools/31.0.0 \
    && mv d8 dx \
    && cd lib  \
    && mv d8.jar dx.jar
```

2. `accessible: module java.base does not "opens java.io" to unnamed module @42760a00`

    É necessário adicionar as seguintes linhas no arquivo `gradle.properties`:

```properties
    #### Obs: versao do Xmx pode variar, use a anterior
    org.gradle.jvmargs=-Xmx1536M \
    --add-exports=java.base/sun.nio.ch=ALL-UNNAMED \
    --add-opens=java.base/java.lang=ALL-UNNAMED \
    --add-opens=java.base/java.lang.reflect=ALL-UNNAMED \
    --add-opens=java.base/java.io=ALL-UNNAMED \
    --add-exports=jdk.unsupported/sun.misc=ALL-UNNAMED \
    --add-exports=jdk.compiler/com.sun.tools.javac.tree=ALL-UNNAMED \
    --add-exports=jdk.compiler/com.sun.tools.javac.code=ALL-UNNAMED \
    --add-exports=jdk.compiler/com.sun.tools.javac.util=ALL-UNNAMED
```


## Referências

[Problema 1](https://stackoverflow.com/questions/68387270/android-studio-error-installed-build-tools-revision-31-0-0-is-corrupted)

[Problema 2](https://stackoverflow.com/questions/67782975/how-to-fix-the-module-java-base-does-not-opens-java-io-to-unnamed-module)

[Tutorial de Setup do MObile-SDK ](https://www.youtube.com/watch?v=6AqgNpili-8)
Obs: ele não resolve os problemas 1 e 2 no vídeo.
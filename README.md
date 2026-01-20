# Embedded application template

## Requirements

None

## Usage

### Windows

- Open VS Code IDE.

- Clone the repository directly in a Dev Container, in the desited volume:
  `Dev Containers: Clone Repository in Named Container Volume`

> [!IMPORTANT]
> Cloning to a container volume in WSL increases massively the performance.

### Linux

- Clone the project:

    ```shell
    git clone https://gitlab.com/catie_estia-students/mbed-os-basic-template.git YOUR_PROJECT_NAME
    cd YOUR_PROJECT_NAME
    ```

- Open the project in VS Code IDE:

    ```shell
    cd YOUR_PROJECT_NAME
    code .
    ```

- Launch the Dev Container:

    ```
    Dev Containers: Reopen in Container
    ```

- In a VS Code terminal, export the project:

    ```shell
    mbed export -i vscode_gcc_arm
    ```

## Build and Flash

- Compile the project:

    ```shell
    mbed deploy # Deploy the dependencies
    mbed compile
    ```

- Program the target device with the compiled binary using a J-Link debug probe:

    ```shell
    sixtron_flash
    ```

## Export to your repository

Modify the remote URL to point to your repository and push the source code:

```shell
git remote set-url origin YOUR_REPOSITORY_URL
git push -u origin main
```

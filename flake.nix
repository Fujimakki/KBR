{
  description = "STM32 Development Environment";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        config.allowUnfree = true;
      };

      stm32cubemx-fixed = pkgs.stm32cubemx.overrideAttrs (oldAttrs: {
        src = pkgs.fetchurl {
          url = "https://sw-center.st.com/packs/resource/library/stm32cube_mx_v6150-lin.zip";
          sha256 = "0asxb6gjg4pxsi54ngigrafpaw2dq8qh0d24rh91sy0siqjna60q"; 
        };
      });

      stm32-clangd = pkgs.writeShellScriptBin "clangd" ''
        exec ${pkgs.clang-tools}/bin/clangd \
          --query-driver="/nix/store/*-arm-none-eabi-gcc*/bin/arm-none-eabi-gcc" \
          "$@"
      '';

    in
    {
      nixpkgs.config.allowUnfree = true;

      devShells.${system}.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          gcc-arm-embedded
          openocd
          stlink
          cmake
          ninja

          stm32cubemx-fixed
 
          # clang-tools
          stm32-clangd
          lldb
          vscode-extensions.vadimcn.vscode-lldb
        ];

        shellHook = ''
          if [ -f .clangd ]; then
            echo "🗑️  Removing broken .clangd file..."
            rm .clangd
          fi

          echo "🛠️ STM32 Dev Environment Activated!"
          echo "Compiler: $(arm-none-eabi-gcc --version | head -n 1)"
        '';
      };
    };
}

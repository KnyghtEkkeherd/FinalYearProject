{
  description = "Example development environment flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        packages = with pkgs; [
          python311
        ];
      in
      {
        devShell = pkgs.mkShell {
          buildInputs = packages;
          shellHook = ''
            echo "Welcome to the development shell!"
            zsh
          '';
        };
      }
    );
}

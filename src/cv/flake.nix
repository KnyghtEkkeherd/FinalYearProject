{
  description = "Example development environment flake";
  # Adapted from the poetry2nix quickstart flake

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    poetry2nix.url = "github:nix-community/poetry2nix";
  };

  outputs = { self, nixpkgs, flake-utils, poetry2nix }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        packages = with pkgs; [ python311 ];
      in
      {
        devShell = pkgs.mkShell {
          buildInputs = packages;
          shellHook = ''
            echo "Welcome to the development shell!"
            python -m venv .venv
            source .venv/bin/activate
            pip install -r requirements.txt
            python -m ipykernel install --user --name=.venv
            zsh
          '';
        };
      });
}

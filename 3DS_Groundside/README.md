# 3DS Groundside

> [!WARNING]
> This is a skeletal project and has no functionality yet.

Groundside operator tooling for 3DS autonomy.

## Layout

```text
3DS_Groundside/
├── src/           # Python package (import src.*)
├── tests/
├── pyproject.toml
└── warg.toml
```

## Modules

- `src/image_stitcher/`: builds an orthomosaic map from drone photos. See
  [its README](src/image_stitcher/README.md).

## Commands

```bash
warg run 3DS_Groundside setup
warg run 3DS_Groundside test
warg run 3DS_Groundside lint
```

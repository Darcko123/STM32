#!/usr/bin/env python3
"""Convierte una imagen a un header .h en formato RGB565 para ILI9341_Disc1.h.

Requiere Pillow:  pip install Pillow

Uso básico:
    python3 image_to_bitmap.py foto.jpg -o foto.h

El resultado es un arreglo `uint32_t` de tamaño IMG_TOTAL_BUF32 (dos píxeles
RGB565 por palabra, little-endian), listo para pasarse directamente a
ILI9341_DisplayImage() sin necesidad de cast.
"""

import argparse
import sys

from PIL import Image, ImageOps

ILI9341_WIDTH = 240
ILI9341_HEIGHT = 320


def rgb888_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def resize_image(img, width, height, mode, bg_color):
    if mode == "stretch":
        return img.resize((width, height), Image.LANCZOS)
    if mode == "crop":
        return ImageOps.fit(img, (width, height), Image.LANCZOS)
    # mode == "fit": conserva aspecto y rellena con bg_color
    fitted = ImageOps.contain(img, (width, height), Image.LANCZOS)
    canvas = Image.new("RGB", (width, height), bg_color)
    offset = ((width - fitted.width) // 2, (height - fitted.height) // 2)
    canvas.paste(fitted, offset)
    return canvas


def image_to_words(img):
    """Devuelve la lista de uint32_t (2 píxeles RGB565 por palabra)."""
    pixels = list(img.getdata())
    px565 = [rgb888_to_rgb565(r, g, b) for r, g, b in pixels]

    if len(px565) % 2:
        px565.append(0)  # relleno si el total de píxeles es impar

    words = []
    for i in range(0, len(px565), 2):
        lo = px565[i]
        hi = px565[i + 1]
        words.append(lo | (hi << 16))
    return words


def write_header(words, width, height, array_name, out_path, source_name):
    guard = array_name.upper() + "_H"
    with open(out_path, "w") as f:
        f.write(f"/* Generado automáticamente por image_to_bitmap.py a partir de '{source_name}'. */\n")
        f.write(f"#ifndef {guard}\n#define {guard}\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define {array_name.upper()}_WIDTH  {width}U\n")
        f.write(f"#define {array_name.upper()}_HEIGHT {height}U\n\n")
        f.write(f"uint32_t {array_name}[{len(words)}] = {{\n")
        for i in range(0, len(words), 12):
            chunk = words[i:i + 12]
            f.write("    " + ", ".join(f"0x{w:08X}" for w in chunk) + ",\n")
        f.write("};\n\n")
        f.write(f"#endif /* {guard} */\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="Ruta de la imagen de entrada")
    parser.add_argument("-o", "--output", help="Ruta del header .h de salida (por defecto: <nombre>.h)")
    parser.add_argument("--name", default="image_data", help="Nombre del arreglo C (por defecto: image_data)")
    parser.add_argument("--width", type=int, default=ILI9341_WIDTH, help="Ancho destino en píxeles (240 en modo retrato)")
    parser.add_argument("--height", type=int, default=ILI9341_HEIGHT, help="Alto destino en píxeles (320 en modo retrato)")
    parser.add_argument("--mode", choices=["fit", "crop", "stretch"], default="fit",
                         help="fit=conserva aspecto y rellena, crop=recorta al centro, stretch=deforma para llenar")
    parser.add_argument("--bg", default="#000000", help="Color de relleno para --mode fit (hex, ej. #000000)")
    parser.add_argument("--rotate", type=int, choices=[0, 90, 180, 270], default=0,
                         help="Rota la imagen antes de redimensionar (útil para orientación landscape)")
    args = parser.parse_args()

    out_path = args.output or (args.input.rsplit(".", 1)[0] + ".h")

    try:
        img = Image.open(args.input).convert("RGB")
    except FileNotFoundError:
        sys.exit(f"Error: no se encontró el archivo '{args.input}'")

    if args.rotate:
        img = img.rotate(-args.rotate, expand=True)

    bg_color = tuple(int(args.bg.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4))

    resized = resize_image(img, args.width, args.height, args.mode, bg_color)
    words = image_to_words(resized)
    write_header(words, args.width, args.height, args.name, out_path, args.input)

    print(f"OK: '{out_path}' generado ({args.width}x{args.height}, {len(words)} palabras uint32_t).")
    print(f"Uso en tu proyecto STM32:\n"
          f'  #include "{out_path.rsplit("/", 1)[-1]}"\n'
          f"  ILI9341_DisplayImage({args.name});")


if __name__ == "__main__":
    main()

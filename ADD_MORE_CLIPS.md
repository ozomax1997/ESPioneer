# Adding clips

1. `pip install pillow numpy`   (video also needs ffmpeg installed)
2. Convert anything:
   ```
   python tools/add_clips.py cool.gif holiday.mp4 logo.png old.lkd
   python tools/add_clips.py my_folder/            # whole folders work
   ```
   Output goes to `sdcard/clips/`. Useful flags: `--auto` (boost dark video), `--gamma 1.4` (brighter),
   `--fit cover` (fill the screen, cropping), `--max-seconds 30`, `--invert`.
3. Copy the new `.dlp` files into the `clips` folder on the microSD card (FAT16 or FAT32). Done: they appear automatically at next boot.

Limits: up to 1500 clips; ~4 KB per frame, so one 20-second clip is about 1.3 MB.
Set `SHUFFLE` to 1 in the sketch for random order.

Card notes: keep clips in the `/clips` folder. Small old cards (<=2 GB) are FAT16, which allows only
~512 entries in the root folder but unlimited files in a subfolder. If the card is blank, format it
FAT16 (or FAT32) with the SD Card Formatter tool, then copy the `clips` folder across.

# zotero


## quick look

https://github.com/mronkko/ZoteroQuickLook/issues/23

```python

sudo apt-get install gnome-sushi

sudo apt-get install gloobus-preview

```

create a bash script:

```bash
#!/bin/bash
# run GNOME sushi

PDF_FILE=$(/usr/bin/realpath "$*")
echo "$PDF_FILE" > test.txt

dbus-send --print-reply --dest=org.gnome.NautilusPreviewer /org/gnome/NautilusPreviewer org.gnome.NautilusPreviewer.ShowFile string:"file://$PDF_FILE" int32:0 boolean:false
```

When you install gloobus-preview, you will have the execute bin file in `/usr/bin/gloobus-preview`, rewrite it with the above script. 
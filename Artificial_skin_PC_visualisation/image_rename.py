import os

i = 1
path = 'img'
name = 'book_667_250x175'

files = os.listdir(path)

for file in files:
    if file.startswith('image'):
        new_name = path + '/' + name + '_' + str(i) + '.png'
        os.rename(path + '/' + file, new_name)
        i += 1

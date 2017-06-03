import urllib.request
import sys
import zipfile

import numpy as np
# import zipfile

class download_package:

    def __init__(self, url):
        print('-- START --')
        self.getURL(url)       #'https://drive.google.com/open?id=0BzObZyY5tpGqc1hZLUxBQlEtTXM'
        print('-- END --')

    # def unarchive():
    #     zipfile

    def getURL(self, url):
        urllib.request.urlretrieve(url, "tmp/downloadedfile.zip")


# def initialize():
#     downloader = download_package()

if __name__ == '__main__':
    # if len(sys.argv) > 1:
    downloader = download_package("downloadedfile3.zip")
    if zipfile.is_zipfile("tmp/downloadedfile.zip"):
        zipf = zipfile.ZipFile("tmp/downloadedfile.zip",'r')
        zipf.extractall("tmp2")
    else:
        print("Illigal Format")
#! /usr/bin/env python

from pyclearsilver import CSPage
import MBPage

class MyPage(MBPage.MBPage):
  def setup(self, hdf):
    pass
    
  def display(self, hdf):
    pass
    
def run(context):
  return MyPage(context, pagename="ping", nologin=False)

def main():
  context = CSPage.Context()
  run(context).start()

if __name__ == "__main__":
  main()

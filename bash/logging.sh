#!/usr/bin/env bash

error() { 
  log 31 "ERROR: $1"; 
  exit 0;
}

warn() { 
  log 33 "WARNING: $1"; 
}

info() { 
  log 32 "INFO: $1"; 
}

log() {
  message="`date +'%Y-%m-%d %H:%M:%S'` $2"
  message="\x1B[${1}m${message}\x1B[0m\n"
  echo -ne "$message"
}
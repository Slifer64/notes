#!/bin/bash
socat pty,link=/tmp/ttyUR,raw,ignoreeof,waitslave tcp:10.0.0.1:54321
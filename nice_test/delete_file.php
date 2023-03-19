<?php
$file = $_GET['file'];
$path = $_GET['path'];

$fullPath = $path . $file;

if (unlink($fullPath)) {
    http_response_code(200);
  } else {
    http_response_code(500);
    error_log('Unable to delete file: ' . $path . $file);
  }
?>
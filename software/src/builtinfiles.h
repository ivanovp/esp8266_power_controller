/**
 * @file builtinfiles.h
 * @brief This file is part of the WebServer example for the ESP8266WebServer.
 *
 * This file contains long, multiline text variables for  all builtin resources.
 */

// used for upload.htm
static const char uploadContent[] PROGMEM =
R"==(
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" lang="en" xml:lang="en">
<head>
  <meta http-equiv="content-type" content="text/html; charset=utf-8"/>
  <meta name="mobile-web-app-capable" content="yes">
  <meta name="viewport" content="user-scalable=no, width=device-width, initial-scale=1.2, maximum-scale=1.2"/>
  <title>Upload</title>
  <link Content-Type="text/css" href="/style.css" rel="stylesheet" />
</head>

<body style="width:300px">
  <h1>Upload files to server</h1>
  <div><a href="/">Index</a></div>
  <div><a href="/admin.htm">Admin</a></div>
  <div><a href="/files.htm">Manage files</a></div>
  <hr>
  <div id='zone' style='width:16em;height:12em;padding:10px;background-color:#ddd'>Drop files here...</div>

  <script>
    // allow drag&drop of file objects
    function dragHelper(e) {
      e.stopPropagation();
      e.preventDefault();
    }

    // allow drag&drop of file objects
    function dropped(e) {
      dragHelper(e);
      var fls = e.dataTransfer.files;
      var formData = new FormData();
      for (var i = 0; i < fls.length; i++) {
        formData.append('file', fls[i], '/' + fls[i].name);
      }
      fetch('/', { method: 'POST', body: formData }).then(function () {
        window.alert('done.');
      });
    }
    var z = document.getElementById('zone');
    z.addEventListener('dragenter', dragHelper, false);
    z.addEventListener('dragover', dragHelper, false);
    z.addEventListener('drop', dropped, false);
  </script>
</body>
)==";

// used when file not found
static const char notFoundContent[] PROGMEM = R"==(
<html>
<head>
  <title>Resource not found</title>
</head>
<body>
  <p>The resource was not found.</p>
  <p><a href="/">Index</a><br>
  <a href="/upload.htm">Upload files to server</a></p>
</body>
)==";

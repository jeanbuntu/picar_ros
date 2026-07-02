// Smooth scroll for in-page quick-nav anchors
document.querySelectorAll('.quick-nav a[href^="#"]').forEach(function (link) {
  link.addEventListener('click', function (e) {
    var target = document.querySelector(link.getAttribute('href'));
    if (!target) return;
    e.preventDefault();
    target.scrollIntoView({ behavior: 'smooth', block: 'start' });
  });
});

// Lazy-load the trajectory viewer iframe only once it scrolls into view,
// since trajviz_out.html pulls in Plotly and is ~800KB.
document.addEventListener('DOMContentLoaded', function () {
  var frame = document.getElementById('trajviz-iframe');
  if (!frame) return;

  var src = frame.getAttribute('data-src');
  if (!('IntersectionObserver' in window)) {
    frame.src = src;
    return;
  }

  var observer = new IntersectionObserver(function (entries) {
    entries.forEach(function (entry) {
      if (entry.isIntersecting) {
        frame.src = src;
        observer.unobserve(frame);
      }
    });
  }, { rootMargin: '200px' });

  observer.observe(frame);
});

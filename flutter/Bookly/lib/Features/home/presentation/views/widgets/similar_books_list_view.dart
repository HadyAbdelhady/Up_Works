import 'package:flutter/material.dart';

import 'custom_book_image.dart';

class SimilarBooksListView extends StatelessWidget {
  const SimilarBooksListView({super.key});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.only(left: 18, right: 18, top: 16),
      child: SizedBox(
        height: MediaQuery.of(context).size.height * 0.15,
        child: ListView.builder(
          padding: EdgeInsets.zero,
          physics: const BouncingScrollPhysics(),
          scrollDirection: Axis.horizontal,
          itemCount: 10,
          itemBuilder: (BuildContext context, int index) {
            return const CustomBookImage(imageUrl: 'https://www.futurepedia.io/_next/image?url=https%3A%2F%2Fcdn.sanity.io%2Fimages%2Fu0v1th4q%2Fproduction%2F0c6d1585cf097da3d56a40c7ae873e7c03a2724a-256x256.svg%3Fw%3D256%26h%3D256%26sharp%3D100%26q%3D100%26auto%3Dformat%26dpr%3D3&w=64&q=75',);
          },
        ),
      ),
    );
  }
}
